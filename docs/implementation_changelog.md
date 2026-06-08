# 执行变更记录：YOLO+ByteTrack / LOS DKF / 时间对齐

日期：2026-06-08

范围：默认 BF PNG 链路：

```text
V4L2 Camera -> RKNN YOLO -> ByteTrack -> StrikePngController
  -> VisualPngGuidance -> BF MSP rate/throttle output
```

本次记录面向已经落地的工程变更，以及仍需实机日志标定的参数项。

## 摘要

- 感知链路使用 YOLO 检测 + 单目标 ByteTrack，降低单帧检测框跳动和短时漏检对控制输入的影响。
- PNG 控制链路接入 LOS 延迟像素卡尔曼滤波，直接估计 `ex/ey/ex_dot/ey_dot`，DKF 失效时自动回退到原差分 LPF。
- 机体去旋转补偿增加飞控角速度历史、视觉曝光时间补偿、串口延迟补偿和插值跨度保护，避免用当前飞控状态补偿旧图像。
- 新增调参、共享内存遥测和 Web plots 字段，用于实机判断 DKF 与 derotation 是否真正改善相位和抖动。

## 已落地变更

### 1. YOLO + ByteTrack 感知链路

- 在共享 BF runtime 配置中保留较低 YOLO 前置置信阈值：
  - `conf_threshold: 0.10`
  - 目的：ByteTrack 需要低分框延续已有轨迹，不能过早在 YOLO/NMS 阶段丢弃。
- 启用单目标 SOT-ByteTrack：
  - `byte_track.enabled: true`
  - 高分主匹配：`high_score_threshold: 0.25`
  - 低分二阶段匹配：`low_score_threshold: 0.10`
  - 新建轨迹阈值：`new_track_threshold: 0.25`
  - IoU 匹配门限：`match_iou_threshold: 0.30`
  - 二阶段 IoU 门限：`second_match_iou_threshold: 0.20`
  - 最大漏检帧数：`max_lost_frames: 5`
- 漏检短时输出 ByteTrack Kalman 预测框：
  - `emit_prediction_on_miss: true`
  - `emit_prediction_max_frames: 5`
  - `max_dt_s: 0.12` 超限后重置轨迹，避免跨大时间间隔错误延续。
- 关闭旧的 detection coast：
  - `detection_coast_s: 0.0`
  - 原因：ByteTrack 已负责短时预测，旧框复用再叠加会造成重复滑行。

相关位置：

- `config/bf_flight_common.yaml`
- `core/include/circle/vision/sot_byte_track.hpp`
- `adapters/bf/bf_runtime/src/bf_runtime_config_yaml.cpp`
- `adapters/bf/bf_runtime/src/bf_control_host.cpp`

### 2. LOS 延迟像素卡尔曼滤波

- `StrikePngController` 复用 `DelayedPixelKalman`，状态为：

```text
[ex, ey, ex_dot, ey_dot]
```

- DKF 使用视觉测量时间戳加入量测：
  - `measurement_ns` 优先来自检测 `capture_ns`
  - `receive_stamp_ns` 使用当前控制时刻 `now_ns`
  - 量测包括 `ex/ey/bbox_area_px/detection_score`
  - 相机内参 `fx/fy` 用于像素噪声到归一化 LOS 噪声的换算。
- 控制时刻预测：
  - `dkf_.predict(input.now_ns, params.dkf)`
  - `predict_extra_delay_s` 用于补偿额外视觉/控制链路延迟。
- DKF 有效性门控：
  - `dkf_enable && dkf.enable`
  - 相机内参有效
  - 估计有效且 `cov_trace <= max_cov_trace`
- DKF 有效时：
  - 控制位置使用 `dkf_est.ex/ey`
  - LOS rate 使用 `dkf_est.ex_dot/ey_dot`
- DKF 关闭或失效时：
  - 保持原有 `ex/ey` 差分 + `pixel_dot_lpf_tau_s` 一阶 LPF 回退路径。
  - 无新量测时继续按 `los_rate_hold_tau_s` 衰减 LOS rate。

默认 PNG 飞行配置已启用：

```yaml
dkf_enable: true
dkf:
  enable: true
  process_accel_noise: 4.0
  meas_noise_px: 4.0
  predict_extra_delay_s: 0.03
  max_cov_trace: 0.25
```

相关位置：

- `core/src/strike_png/strike_png_controller.cpp`
- `core/include/circle/strike_png/strike_png_controller.hpp`
- `core/include/circle/strike/delayed_pixel_kalman.hpp`
- `core/src/strike/delayed_pixel_kalman.cpp`
- `config/strike_png_bf_flight*.yaml`

### 3. 机体去旋转时间对齐

- `PngControllerAdapter` 新增机体角速度历史缓存：
  - 样本字段：`stamp_ns/roll_rate_rad_s/pitch_rate_rad_s/yaw_rate_rad_s/valid`
  - 缓存大小：256 个样本
  - 保留时间：最多约 2 秒
- 当前 BF 链路没有直接输入实测 body rate，本版先用 `MSP_ATTITUDE` 姿态角差分生成角速度：
  - roll/pitch/yaw 都做角度 wrap 处理
  - 后续接入 BF gyro 或 `MSP_RAW_IMU` 后可替换为实测角速度。
- 飞控样本写入时间补偿：

```text
sample.stamp_ns = vehicle.stamp_ns - fc_serial_latency_ns
```

- 视觉查询时间补偿：

```text
lookup_ns = input.measurement_ns - camera_exposure_midpoint_offset_ns
```

- 历史查询逻辑：
  - 查询时间早于最老样本或晚于最新样本：invalid
  - 查询时间落在两个样本之间：线性插值
  - 如果样本跨度超过 `max_derotate_interpolation_gap_s`：invalid
- 去旋转补偿现在受 `derotate_rate_valid` 门控：
  - `derotate_body_rates=true` 且历史查询 valid 时才补偿
  - 查询 invalid 时跳过机体去旋转，直接使用 DKF/LPF 输出的 LOS rate
  - 遥测丢包或插值跨度过大时不会错误插值产生尖峰。

新增参数：

```yaml
derotate_history_enable: true
camera_exposure_midpoint_offset_ns: 0
fc_serial_latency_ns: 0
max_derotate_interpolation_gap_s: 0.02
body_rate_observer_enable: false
```

飞行配置中 `derotate_body_rates` 仍保持 `false`，因此默认不会改变现有飞行行为；只有显式打开 derotation 后，历史对齐链路才参与补偿。

相关位置：

- `adapters/bf/bf_flight_png/src/png_controller_adapter.cpp`
- `adapters/bf/bf_flight_png/src/png_controller_adapter.hpp`
- `adapters/bf/common/src/bf_fc_adapters.cpp`
- `core/src/strike_png/visual_png_guidance.cpp`
- `core/include/circle/strike_png/visual_png_guidance.hpp`

### 4. 调参、遥测与 Web 调试

新增或扩展的调试字段：

```text
derotate_lookup_valid
derotate_lookup_age_ms
derotate_interp_gap_ms
derotate_roll_rate_rad_s
derotate_pitch_rate_rad_s
camera_exposure_midpoint_offset_ns
fc_serial_latency_ns
```

用途：

- 判断历史样本是否稳定查到。
- 判断插值跨度是否大多数小于 20 ms。
- 观察 derotation 开启后 roll/pitch 前馈是否出现尖峰。
- 比较不同 offset 参数下，控制命令相位是否更贴近目标运动。

其他工程变更：

- `StrikeTelemetry` 共享内存版本更新到 v5，携带新增 PNG derotation 字段。
- `bf_debugd` 参数快照和在线调参支持新增 DKF/derotation 参数。
- Web plots 增加 derotation lookup、插值跨度和历史角速度曲线。

相关位置：

- `adapters/common/ipc/include/circle/ipc/strike_telemetry_shm.hpp`
- `adapters/common/ipc/src/strike_telemetry_shm.cpp`
- `adapters/common/debug/src/strike_png_param_tune.cpp`
- `adapters/common/debug/include/circle/debug/tracking_debug_schema.hpp`
- `adapters/common/debug/src/tracking_debug_series.cpp`
- `adapters/common/debug/web/plots.html`

### 5. 测试覆盖

新增/扩展测试覆盖：

- ByteTrack runtime YAML 配置解析：
  - `enabled`
  - high/low/new track thresholds
  - IoU matching thresholds
  - miss prediction 参数
  - `detection_coast_s: 0.0`
- PNG DKF 参数解析：
  - `dkf_enable`
  - `process_accel_noise`
  - `meas_noise_px`
  - `predict_extra_delay_s`
  - `max_cov_trace`
- derotation 历史时间补偿：
  - `fc_serial_latency_ns` 写入补偿
  - `camera_exposure_midpoint_offset_ns` 查询补偿
  - 插值结果和 lookup age 输出
- derotation 大跨度保护：
  - 遥测样本间隔 50 ms、最大允许 20 ms 时，查询 invalid。
- 共享内存 JSON 输出：
  - 新增 derotation 字段可序列化。
- 在线调参 round trip：
  - DKF 参数
  - derotation history 参数
  - 时间 offset 参数

相关位置：

- `adapters/bf/tests/bf_runtime_test.cpp`
- `core/tests/smoke_test.cpp`

## 验证记录

已执行并通过的构建/测试：

```bash
cmake -S adapters/bf -B adapters/bf/build_codex2 \
  -DCIRCLE_BF_BUILD_TESTS=ON \
  -DCIRCLE_STRIKE_BUILD_TESTS=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
cmake --build adapters/bf/build_codex2 -j
ctest --test-dir adapters/bf/build_codex2 --output-on-failure \
  -R 'circle_strike_core_smoke_test|bf_runtime_test'
```

结果：

- `circle_strike_core_smoke_test` 通过。
- `bf_runtime_test` 通过。
- `git diff --check` 通过。

已知测试状态：

- 全量 `ctest --test-dir adapters/bf/build_codex2 --output-on-failure` 中 `bf_params_parse_test` 仍有旧期望值不匹配，主要是 `strike_bf_flight.yaml` 的串口、rate/channel 等配置期望滞后；该失败与本次 PNG DKF/ByteTrack/derotation 改动无直接关系。

## 实机标定建议

建议按三组日志对比：

1. DKF 开启，derotation 关闭。
2. DKF 开启，derotation 开启，offset 全 0。
3. DKF 开启，derotation 开启，逐步调：
   - `fc_serial_latency_ns`: 2-5 ms 起试
   - `camera_exposure_midpoint_offset_ns`: 0-10 ms 起试

验收指标：

- `derotate_lookup_valid` 大多数帧为 true，遥测丢包时能自动变 false。
- `derotate_interp_gap_ms` 大多数小于 `20 ms`。
- derotation 打开后，`png_ff_roll_rad_s` / `png_ff_pitch_rad_s` 不出现新增高频尖峰。
- 调整 offset 后，末端指令相位更贴近目标运动方向，目标出框趋势降低。

## 已知限制与下一步

- 当前 BF body rate 来源仍是 `MSP_ATTITUDE` 姿态差分，不是实测 gyro。下一步建议接入 BF gyro 或 `MSP_RAW_IMU`。
- `body_rate_observer_enable` 已作为参数预留，但首版默认关闭，避免在时间对齐尚未实测前引入额外相位滞后。
- 本次未实现杆臂效应补偿；当前只处理机体角速度去旋转的时间对齐。
- derotation 默认未打开，实机前应先用日志确认历史查询质量，再逐步开启。
- `bf_params_parse_test` 的旧配置期望需要单独更新，避免长期掩盖真实配置解析回归。
