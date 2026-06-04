# 算法与工程流程总览

本文档按实机使用顺序重新梳理 `circle_ai_strike`：先说明当前默认模式、编译、启动、调试和测试流程，再说明 PNG 上层视觉算法的原理、源码实现和调参顺序。

## 1. 当前默认结论

当前默认启动链路是 `bf_flight_png`，也就是 `StrikePngController` / `VisualPngGuidance` 这条 PNG 比例导引链路。启动脚本 `scripts/launch/launch_bf.sh` 中 `PNG=1`，默认使用：

```text
adapters/bf/build/bf_flight_png
adapters/bf/build/bf_debugd
config/strike_png_bf_flight.yaml
config/strike_png_bf_debug.yaml
```

传统 `bf_flight` 仍然存在，但只有在脚本改为 `PNG=0` 或手动运行 `adapters/bf/build/bf_flight` 时使用。两条链路共享 MSP 串口、相机、推理、共享内存和 Web 调试服务，运行时互斥，不能同时控制同一架飞机。

注意：当前 `config/strike_png_bf_flight.yaml` 里 `dry_run: false`。台架和首次联调前，应先使用临时 dry-run 配置或在线参数把 `target_strike_png.dry_run` 设为 `true`，确认方向、油门和模式门控都正确后再允许实际输出。

## 2. 仓库结构与入口

当前 checkout 的真实路径是仓库根目录下的这些目录；README 里仍保留了旧工作空间 `src/circle_pilot/...` 的描述，读代码时以当前路径为准。

```text
adapters/bf/                     # BF 二进制、runtime、CMake 入口
adapters/bf/bf_flight_png/       # PNG 飞行进程入口和 BF adapter
adapters/bf/bf_flight/           # 传统 StrikeController 飞行进程
adapters/bf/bf_debugd/           # Web 调试/监控服务
adapters/common/                 # IPC、debug、perception 公共库
core/                            # 控制算法、参数解析、视觉后处理
config/                          # 飞行、debug、相机和模型配置
scripts/launch/                  # BF 双进程启动脚本
circle_pilot_bringup/config/     # 相机内参
circle_pilot_object_tracking/    # RKNN/YOLO 模型资产
```

主要源码入口：

```text
adapters/bf/bf_flight_png/src/main.cpp
adapters/bf/bf_flight_png/src/png_controller_adapter.cpp
core/src/strike_png/strike_png_controller.cpp
core/src/strike_png/visual_png_guidance.cpp
core/src/strike_png/strike_png_params_yaml.cpp
```

## 3. 编译流程

### 3.1 编译环境

最低要求：

- CMake 3.16 或更高。
- 支持 C++17 的编译器。
- `Eigen3`，这是 `core` 的必需依赖。
- `yaml-cpp`，用于 YAML 参数加载；缺失时可以编译部分库，但实机配置加载和调参能力会受限。
- `OpenCV`，用于 `bf_debugd` 图像编码、标注帧保存等 debug 能力。

实机图像链路还需要目标板上的 Rockchip MPP/RGA/RKNN 运行库、相机设备、模型文件和 MSP 串口设备。WebRTC 预览需要 GStreamer WebRTC 相关开发包和运行时插件；缺失时可以退回 MJPEG。

### 3.2 标准编译

从仓库根目录编译 BF 实机栈：

```bash
cmake -S adapters/bf -B adapters/bf/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build adapters/bf/build -j
```

只编主要产物也可以指定 target：

```bash
cmake --build adapters/bf/build --target bf_flight_png bf_debugd -j
```

预期产物：

```text
adapters/bf/build/bf_flight_png
adapters/bf/build/bf_flight
adapters/bf/build/bf_debugd
```

`bf_flight_png` 是默认 PNG 控制进程；`bf_debugd` 是地面监控和在线调参进程；`bf_flight` 是传统状态机控制进程。

### 3.3 带测试编译

同时打开 core 和 BF adapter 测试：

```bash
cmake -S adapters/bf -B adapters/bf/build \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCIRCLE_BF_BUILD_TESTS=ON \
  -DCIRCLE_STRIKE_BUILD_TESTS=ON
cmake --build adapters/bf/build -j
```

运行全部 CTest：

```bash
ctest --test-dir adapters/bf/build --output-on-failure
```

按名称运行部分测试：

```bash
ctest --test-dir adapters/bf/build -R circle_strike_core_smoke_test --output-on-failure
ctest --test-dir adapters/bf/build -R bf_params_parse_test --output-on-failure
ctest --test-dir adapters/bf/build -R bf_runtime_test --output-on-failure
```

如果 `yaml-cpp` 没有被 CMake 找到，YAML 参数解析相关测试可能不会生成。此时应先修复依赖，而不是跳过配置解析验证。

### 3.4 只编 core 算法

只修改 `core` 算法时，可以更快地单独编译核心库：

```bash
cmake -S core -B core/build -DCIRCLE_STRIKE_BUILD_TESTS=ON
cmake --build core/build -j
ctest --test-dir core/build --output-on-failure
```

这会覆盖 `StrikePngController`、传统 `StrikeController`、视觉后处理和参数解析相关的核心测试。

### 3.5 重新配置与常见问题

如果切换依赖或 CMake option 后行为异常，重新生成 build 目录：

```bash
rm -rf adapters/bf/build
cmake -S adapters/bf -B adapters/bf/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build adapters/bf/build -j
```

常见问题：

- `Eigen3` 找不到：安装 Eigen 开发包或设置 `CMAKE_PREFIX_PATH`。
- YAML 参数没有生效：确认 `yaml-cpp` 被找到，构建日志中应出现 `CIRCLE_STRIKE_HAS_YAML=1` 相关编译定义。
- `bf_debugd` 页面无视频：先查 `/api/video/status`，再确认 `preview_shm`、OpenCV、GStreamer/WebRTC 或 MJPEG 配置。
- 实机无法打开相机/模型：检查 `config/bf_flight_common.yaml`、相机 YAML、RKNN 模型路径、设备权限和目标板库路径。

## 4. 启动与部署流程

### 4.1 启动前检查

上机前至少确认：

1. `adapters/bf/build/bf_flight_png` 和 `bf_debugd` 已存在且可执行。
2. `config/strike_png_bf_flight.yaml` 中 MSP 串口、相机、模型、RC 映射和油门档位符合当前飞机。
3. `dry_run` 阶段已验证 `ex/ey` 符号、roll/pitch 输出方向、油门归一化和 BF mode override。
4. Betaflight 已配置可接受 MSP override 或对应 RC override 模式。
5. 目标板上 RGA/MPP/RKNN、相机设备、串口权限和 NPU 权限正常。

### 4.2 查看启动脚本帮助

在开发机或台架上只查看参数，不触发 performance 调频：

```bash
SKIP_PERFORMANCE=1 scripts/launch/launch_bf.sh --help
```

### 4.3 启动 PNG 双进程

默认启动飞行进程和 debug 服务：

```bash
scripts/launch/launch_bf.sh --png
```

只启动主控：

```bash
scripts/launch/launch_bf.sh --png --flight
```

只启动 Web 调试服务：

```bash
scripts/launch/launch_bf.sh --png --debug
```

指定配置文件：

```bash
scripts/launch/launch_bf.sh --png \
  --flight-config config/strike_png_bf_flight.yaml \
  --debug-config config/strike_png_bf_debug.yaml
```

也可以直接运行二进制，便于在调试器或 systemd 中接管：

```bash
adapters/bf/build/bf_flight_png -c config/strike_png_bf_flight.yaml
adapters/bf/build/bf_debugd -c config/strike_png_bf_debug.yaml
```

脚本会把 `CIRCLE_PILOT_ROOT` 设为仓库根目录。日志目录来自 `scripts/launch/launch_common.sh` 的 `WS_ROOT/logs_ws`；在当前抽取仓库布局下，`WS_ROOT` 可能解析到仓库上层，而不是仓库本身。启动输出会打印真实日志路径，应以该输出为准。

### 4.4 dry-run 建议流程

当前默认 YAML 不是 dry-run。建议台架使用临时配置：

```bash
cp config/strike_png_bf_flight.yaml /tmp/strike_png_dryrun.yaml
```

然后把 `/tmp/strike_png_dryrun.yaml` 中：

```yaml
strike_png:
  dry_run: true
```

再启动：

```bash
scripts/launch/launch_bf.sh --png --flight-config /tmp/strike_png_dryrun.yaml
```

dry-run 下算法仍会计算检测、误差、PNG 输出和遥测，但 runtime 不应向 Betaflight 写实际算法控制量。确认安全门控、图像和输出方向后，再切回正式配置。

## 5. 调试与监控流程

### 5.1 Web 监控入口

`bf_debugd` 默认监听 `config/strike_png_bf_debug.yaml` 中的：

```yaml
bf_debug:
  http_port: 8080
  tune_mode: target_strike_png
```

浏览器访问：

```text
http://<drone-ip>:8080/
http://<drone-ip>:8080/plots
```

常用 HTTP API：

```text
GET  /api/series.json          # 曲线数据
GET  /api/series/session       # 曲线 EventSource
GET  /api/video/status         # 视频/WebRTC/MJPEG 状态
GET  /api/video/session        # 视频状态 EventSource
GET  /api/video/mjpeg          # MJPEG 预览
GET  /api/params.json          # 可调参数快照
POST /api/param                # 在线修改单个参数
POST /api/params/save          # 保存当前可调参数
GET  /api/params/save/download # 下载保存后的参数文件
POST /api/webrtc/offer         # WebRTC SDP offer
```

示例：

```bash
curl "http://<drone-ip>:8080/api/video/status"
curl "http://<drone-ip>:8080/api/params.json"
curl -X POST "http://<drone-ip>:8080/api/param" \
  -H "Content-Type: application/json" \
  -d '{"name":"target_strike_png.nav_ratio_x","value":2.0}'
```

调参接口接受 `target_strike_png.` 前缀，也接受较短的 `strike_png.` 前缀。在线调参适合小步验证；稳定值应写回 YAML，并重新走 dry-run 验证。

### 5.2 共享内存与日志

飞行进程写入：

```text
/circle_pilot_strike_telemetry
/circle_pilot_debug_preview
/circle_pilot_strike_params_snapshot
```

`bf_debugd` 读取这些 shared memory 并生成 Web 曲线、预览视频、参数表和图像落盘。调试时同时查看：

- 启动脚本打印的 `bf_flight_png` 日志。
- 启动脚本打印的 `bf_debugd` 日志。
- Web 页面曲线。
- `/api/video/status` 的编码器、预览帧和 WebRTC 状态。
- `logs_ws/debug_frames` 下的 raw/annotated 图像，前提是 `image_save.enabled: true`。

### 5.3 推荐调试顺序

1. 编译后先运行 `SKIP_PERFORMANCE=1 scripts/launch/launch_bf.sh --help`，确认脚本路径和默认配置。
2. 只启动 `bf_debugd`，确认 8080 页面能打开。
3. 用 dry-run 配置启动 `bf_flight_png`，确认相机、模型、检测、共享内存和曲线都有数据。
4. 移动目标，检查 `vision_ex`、`vision_ey` 是否随目标方向变化。
5. 检查 `png_ff_*`、`png_fov_trim_*`、`png_intercept_*` 与最终 `vision_roll_sp_rad`、`vision_pitch_sp_rad` 的关系。
6. 上电但不解锁时确认 `require_armed_to_command` 生效。
7. 解锁前确认 `hover_thrust_z`、`strike_thrust_z` 是当前飞机可接受的归一化杆量。
8. 小步调整 `nav_ratio_x/y`、`closure_*`、`max_*_rate_rad_s`，每次只改一类参数。

### 5.4 关键遥测字段

通用字段：

```text
has_target
detection_valid
detection_score
bbox_area_ratio
vision_ex
vision_ey
vision_roll_sp_rad
vision_pitch_sp_rad
vision_thrust_z
vehicle_roll_rad
vehicle_pitch_rad
vehicle_throttle_algo_norm
vehicle_throttle_cmd_norm
armed
msp_override_active
```

PNG 专用字段：

```text
png_closure_scale
png_ex_dot_inertial
png_ey_dot_inertial
png_measurement_age_s
png_ff_roll_rad_s
png_ff_pitch_rad_s
png_fov_trim_roll_rad_s
png_fov_trim_pitch_rad_s
png_edge_guard_roll_rad_s
png_edge_guard_pitch_rad_s
png_pursuit_roll_rad_s
png_pursuit_pitch_rad_s
png_stale_trim_roll_rad_s
png_intercept_roll_rad_s
png_intercept_pitch_rad_s
png_crossing_pitch_rad_s
png_future_ex
png_future_ey
png_intercept_active
png_crossing_active
png_fwd_guard_active
png_loss_hold_latched
png_tilt_hardcap_active
png_entry_handoff_progress
```

## 6. 测试流程

### 6.1 自动化测试

推荐在每次修改算法或 runtime 后执行：

```bash
cmake -S adapters/bf -B adapters/bf/build \
  -DCIRCLE_BF_BUILD_TESTS=ON \
  -DCIRCLE_STRIKE_BUILD_TESTS=ON
cmake --build adapters/bf/build -j
ctest --test-dir adapters/bf/build --output-on-failure
```

测试覆盖重点：

- `circle_strike_core_smoke_test`：核心控制算法、PNG guidance、传统 strike 模块的 smoke test。
- `bf_params_parse_test`：YAML 参数解析覆盖，防止配置项漏读或名称漂移。
- `bf_runtime_test`：BF runtime、传统 adapter、PNG adapter、共享参数更新和部分控制输出行为。

### 6.2 手动功能测试

自动化测试不连接真实硬件，因此上机前还需要手动检查：

1. `bf_flight_png -c <dry-run-yaml>` 能启动并持续运行。
2. `/api/series.json` 中 `detection_valid`、`vision_ex/ey`、`bbox_area_ratio` 合理。
3. `/api/video/status` 显示预览帧更新。
4. Web 参数修改后，`/api/params.json` 能读回新值。
5. MSP override 未激活或未 armed 时，runtime 不发布实际算法命令。
6. 目标短时丢失时，`png_loss_hold_latched` 和 physical hold 行为符合预期。
7. 倾角接近上限时，`png_tilt_hardcap_active` 或 softcap 系数能正确出现。

### 6.3 回归测试建议

只改文档不需要运行构建。只改参数 YAML 时至少运行：

```bash
cmake --build adapters/bf/build --target bf_params_parse_test -j
ctest --test-dir adapters/bf/build -R bf_params_parse_test --output-on-failure
```

只改 `core/src/strike_png` 时至少运行：

```bash
ctest --test-dir adapters/bf/build -R circle_strike_core_smoke_test --output-on-failure
ctest --test-dir adapters/bf/build -R bf_runtime_test --output-on-failure
```

只改 `adapters/bf/bf_flight_png` 或 runtime 时至少运行：

```bash
ctest --test-dir adapters/bf/build -R bf_runtime_test --output-on-failure
```

## 7. 总体控制数据流

默认 PNG 实机链路：

```text
V4L2 相机
  -> MPP/RGA 图像预处理
  -> RKNN YOLO 推理
  -> NMS 与目标过滤
  -> 图像误差 ex/ey、目标框面积、检测时间戳
  -> PngControllerAdapter
  -> StrikePngController
  -> VisualPngGuidance
  -> roll/pitch/yaw rate + thrust_z
  -> BF runtime 安全门控、油门交接、物理保持
  -> MSP SET_RAW_RC
  -> Betaflight 飞控
```

`bf_debugd` 与控制链路旁路并行，不直接控制飞机；它只读取共享内存、显示曲线/视频、提供参数调节入口。

## 8. PNG 算法输入

检测框中心为 `(det.cx, det.cy)`，相机内参光心为 `(intr.cx, intr.cy)`，焦距为 `(intr.fx, intr.fy)`。归一化图像误差：

```text
ex = (det.cx - intr.cx) / intr.fx
ey = (det.cy - intr.cy) / intr.fy
```

含义：

- `ex > 0`：目标在图像中心右侧。
- `ey > 0`：目标在图像中心下方。
- `bbox_area_ratio = det.width * det.height / image_area`：用目标框面积近似接近程度。

`PngControllerAdapter` 还会传入：

```text
roll_rate_rad_s
pitch_rate_rad_s
vehicle_roll_rad
vehicle_pitch_rad
ownship_forward_speed_m_s
measurement_ns
now_ns
```

检测新鲜度由 `detection_stale_s` 限制。过期检测不会作为新目标输入控制器。

## 9. PNG 算法原理

### 9.1 像素速度与延迟补偿

`StrikePngController` 用相邻测量计算图像误差速度：

```text
ex_dot_raw = (ex_now - ex_prev) / dt
ey_dot_raw = (ey_now - ey_prev) / dt
```

再做一阶低通：

```text
alpha = dt / (pixel_dot_lpf_tau_s + dt)
ex_dot_filt += alpha * (ex_dot_raw - ex_dot_filt)
ey_dot_filt += alpha * (ey_dot_raw - ey_dot_filt)
```

如果控制周期没有新检测，不会立刻清零 LOS rate，而是按 `los_rate_hold_tau_s` 指数衰减旧速度，避免低检测帧率导致指令断续。

检测有时间戳延迟时，视觉预测会外推当前控制误差：

```text
control_ex = ex + ex_dot_filt * measurement_age_s
control_ey = ey + ey_dot_filt * measurement_age_s
```

外推由这些参数限制：

```yaml
visual_prediction_enable
visual_prediction_max_age_s
visual_prediction_max_offset_norm
```

### 9.2 PNG 比例导引前馈

经典 PNG 的思想是让速度方向角变化率跟随视线角变化率：

```text
d_sigma / dt = N * d_q / dt
```

当前工程在图像域近似实现：`ex_dot/ey_dot` 近似视线角速度，`bbox_area_ratio` 近似接近强度。闭合率缩放：

```text
closure_scale = closure_base_scale + closure_area_gain * sqrt(bbox_area_ratio)
```

PNG 前馈：

```text
roll_ff  = nav_ratio_x * closure_scale * ex_dot_inertial
pitch_ff = -nav_ratio_y * closure_scale * ey_dot_inertial
```

关键参数：

```yaml
nav_ratio_x
nav_ratio_y
closure_base_scale
closure_area_gain
max_feedforward_rad_s
```

`nav_ratio_x/y` 越大，越积极响应视线角速度；`max_feedforward_rad_s` 防止检测跳变或末端噪声造成过大角速度。

### 9.3 机体角速度去旋转

相机固定在机体上，机体自身转动也会造成目标在画面中移动。去旋转用于从像素速度里扣除机体转动影响：

```text
ex_dot_inertial = ex_dot + pitch_rate * derotate_pitch_to_x_gain
ey_dot_inertial = ey_dot - roll_rate * derotate_roll_to_y_gain
```

对应参数：

```yaml
derotate_body_rates
derotate_pitch_to_x_gain
derotate_roll_to_y_gain
residual_rate_limit_rad_s
```

`residual_rate_limit_rad_s` 会限制去旋转后的残差速度，避免异常角速度或误检放大成控制突变。

### 9.4 PNG + P/PD 修正结构

上层视觉算法不是完整 PID。默认 PNG 模式可以理解为：

```text
rate_cmd =
  PNG 前馈
  + FOV trim P 修正
  + pursuit fallback P 修正
  + edge guard P 修正
  + terminal stale trim P 修正
  + terminal intercept P 修正
  + terminal crossing D 阻尼
```

没有主链路积分项 `I`。目标检测存在延迟、丢帧和机动，积分容易 wind-up；慢速偏差由 P 项处理，运动趋势由 PNG/D 项处理，底层姿态/角速度 PID 由 Betaflight 自身完成。

### 9.5 FOV trim 与 pursuit fallback

PNG 前馈依赖视线速度。如果目标偏心但运动慢，PNG 输出可能很小，因此叠加 FOV trim：

```text
roll_trim  = fov_trim_kp_rate * ex_error
pitch_trim = -fov_trim_kp_rate * ey_error
```

FOV trim 可随目标面积增大淡出：

```text
fov_trim_scale = 1 - smoothstep((bbox_area_ratio - fade_start) / fade_range)
```

pursuit fallback 是 PNG 弱时才介入的保底比例追踪：

```text
if abs(png_command) <= pursuit_fallback_png_weak_rate:
    fallback = kp * error * smoothstep(error_range)
```

它避免目标明显偏离中心但 LOS rate 很小时控制器无动作。

### 9.6 边缘保护

目标靠近图像边缘时，edge guard 优先把目标拉回视场：

```text
weight = smoothstep((abs(error) - edge_guard_start_norm) /
                    (edge_guard_full_norm - edge_guard_start_norm))
command = edge_guard_kp_rate * error * weight
```

对应参数：

```yaml
edge_guard_enable
edge_guard_start_norm
edge_guard_full_norm
edge_guard_kp_rate
edge_guard_min_rate_rad_s
edge_guard_max_rate_rad_s
edge_guard_pitch_scale
```

它不是正常阶段主导引，只在接近出框时明显生效。

### 9.7 末端补偿

末端补偿依赖 `bbox_area_ratio` 打开，目标越近权重越高。

terminal stale lateral trim：检测过期且目标较近时，按横向误差继续给保守 roll 修正：

```text
roll_stale_trim = kp * ex * area_weight * stale_weight
```

terminal intercept：预测未来误差并做 P 修正：

```text
future_ex = ex + ex_dot_inertial * terminal_intercept_lead_s
future_ey = ey + ey_dot_inertial * terminal_intercept_lead_s
roll_intercept  = terminal_intercept_kp_rate * future_ex * weight
pitch_intercept = -terminal_intercept_kp_rate * future_ey * weight
```

terminal crossing：对纵向视线速度做 D 型阻尼：

```text
pitch_crossing = -terminal_crossing_kd_rate * ey_dot_inertial * weight
```

terminal forward speed guard：目标近且前向速度高时，缩放正向 pitch 命令，避免末端继续加速或过冲。

### 9.8 最终叠加和符号映射

roll 方向大致合成：

```text
roll =
  roll_png_ff
  + roll_fov_trim * fov_trim_scale
  + roll_pursuit_fallback
  + roll_edge_guard
  + roll_terminal_stale_trim
  + roll_terminal_intercept
```

pitch 方向大致合成：

```text
pitch =
  pitch_png_ff
  + pitch_fov_trim * fov_trim_scale
  + pitch_pursuit_fallback
  + pitch_edge_guard
  + pitch_terminal_intercept
  + pitch_terminal_crossing
```

随后限幅：

```text
roll  = clamp(roll,  -max_roll_rate_rad_s,  max_roll_rate_rad_s)
pitch = clamp(pitch, -max_pitch_rate_rad_s, max_pitch_rate_rad_s)
```

最后通过 `lateral_output_sign` 和 `longitudinal_output_sign` 映射到 Betaflight 的坐标和相机安装约定。方向错误会导致目标越修越偏，这是首轮 dry-run 必查项。

## 10. PNG 源码实现方式

### 10.1 文件职责

```text
adapters/bf/bf_flight_png/src/main.cpp
  解析 -c/--config，加载 PNG 参数和 BF runtime 配置，创建 BfControlHost。

adapters/bf/bf_flight_png/src/png_controller_adapter.cpp
  把 BF runtime context 转成 StrikePngInput；处理油门、entry handoff、
  target loss hold、遥测填充和在线参数更新。

core/src/strike_png/strike_png_controller.cpp
  维护跨周期状态；计算像素速度、LOS rate hold、视觉预测；调用 guidance；
  最后应用 TiltEnvelope。

core/src/strike_png/visual_png_guidance.cpp
  实现 PNG 前馈、P/PD 修正、末端补偿、限幅和符号映射。

core/src/strike_png/strike_png_params_yaml.cpp
  读取 YAML 并 clamp 参数范围。
```

对应结构体在：

```text
core/include/circle/strike_png/strike_png_node_params.hpp
core/include/circle/strike_png/strike_png_controller.hpp
core/include/circle/strike_png/visual_png_guidance.hpp
```

### 10.2 单周期调用流程

`PngControllerAdapter::update()`：

```text
读取 ctx.intrinsics / ctx.detection / ctx.vehicle
检查相机内参和 detection_stale_s
如果检测有效：
  计算 ex / ey / bbox_area_ratio / measurement_ns
复制 roll_rate / pitch_rate / attitude / forward_speed
调用 StrikePngController::tick(params.controller, input)
根据 has_target 选择 strike_thrust_z 或 hover_thrust_z
应用 entry handoff 平滑
更新 target loss hold
返回 BfControlResult 给 BF runtime
```

`StrikePngController::tick()`：

```text
如果未启用或没有目标：
  reset()
  返回无目标输出

计算 ctrl_dt_s 和 measurement_age_s
如果有新测量：
  差分 ex/ey 得到 raw dot
  按 pixel_dot_lpf_tau_s 低通
否则：
  按 los_rate_hold_tau_s 衰减旧 dot

可选 visual prediction
调用 VisualPngGuidance::compute()
可选 TiltEnvelope 姿态包络
返回 roll/pitch rate 和 PNG 调试量
```

`VisualPngGuidance::compute()`：

```text
ex_dot/ey_dot
  -> body-rate derotation
  -> residual rate clamp
  -> closure_scale
  -> PNG feedforward
  -> FOV trim / fallback / edge guard
  -> terminal stale/intercept/crossing/forward-speed guard
  -> max rate clamp
  -> output sign mapping
```

## 11. Runtime 安全门控

控制器只生成期望 `RateCommand`，是否真的写到 Betaflight 由 BF runtime 决定。主要门控：

- `mode_active`：MSP override 模式或 dry-run 是否激活。
- `dry_run`：只计算和发布遥测，不应输出实际算法控制。
- `require_armed_to_command`：未解锁时禁止算法命令。
- watchdog：飞控状态停滞或控制循环超时时封锁输出。
- `engage_require_fresh_detection`：接管后是否必须先看到新检测。
- `detection_coast_s`：短时无检测时是否允许复用上一帧。
- throttle handover：算法油门与物理油门之间的线性过渡。

runtime 输出模式可理解为：

```text
Algorithm     -> 发布算法 roll/pitch/yaw/throttle
LevelOnly     -> 只发布回平命令
PhysicalHold  -> 保持物理杆量或 latched throttle
None          -> 不让算法拥有控制权
```

PNG 控制器自己的姿态保护由 `tilt_cap` 完成。softcap 在姿态接近上限时压制继续增大倾角方向的角速度；hardcap 超过阈值后输出回平角速度：

```text
rate_level = -hardcap_level_kp * attitude
```

## 12. PNG 参数调试顺序

推荐按风险从低到高调：

1. 检测链路：`target_class_name`、`min_score`、`detection_stale_s`。
2. 输出安全：`dry_run`、`require_armed_to_command`、`max_roll_rate_rad_s`、`max_pitch_rate_rad_s`。
3. 油门：`hover_thrust_z`、`strike_thrust_z`、`entry_smooth_initial_thrust_z`。
4. 坐标方向：`lateral_output_sign`、`longitudinal_output_sign`。
5. 像素速度：`pixel_dot_lpf_tau_s`、`los_rate_hold_tau_s`、`visual_prediction_*`。
6. PNG 主响应：`nav_ratio_x/y`、`closure_base_scale`、`closure_area_gain`、`max_feedforward_rad_s`。
7. 居中和保底：`fov_trim_kp_rate`、`fov_trim_fade_*`、`pursuit_fallback_*`。
8. 视场保护：`edge_guard_*`。
9. 末端行为：`terminal_stale_*`、`terminal_intercept_*`、`terminal_crossing_*`、`terminal_forward_speed_guard_*`。
10. 姿态安全：`tilt_cap.*`。

调参原则：

- 每次只改一类参数，保留日志和曲线截图。
- 先降低上限确认方向，再逐步提高响应。
- 先调横向 roll，再调纵向 pitch；pitch 还会受油门、前向速度和相机安装影响。
- 检测抖动明显时，优先调 `pixel_dot_lpf_tau_s` 和检测阈值，不要直接提高 `nav_ratio`。
- 末端过冲先看 `terminal_crossing_*` 和 `terminal_forward_speed_guard_*`，不要只靠降低全局 rate 上限解决。

## 13. 传统 bf_flight / StrikeController

传统 `bf_flight` 使用 `StrikeController`，是更完整的状态机式控制栈。状态包括：

```text
WaitingTarget
Tracking
ForceLevel
CommitHold
FaFallback
Complete
```

传统 tracking 主项更接近图像 PD：

```text
roll_cmd =
  lateral_output_sign *
  (lateral_kp_rate * ex + lateral_kd_rate * ex_dot)

pitch_cmd =
  longitudinal_output_sign *
  (longitudinal_kp_rate * ey + longitudinal_kd_rate * ey_dot)
```

它还会叠加或串联：

- `DelayedPixelKalman`：检测延迟和短时过期预测。
- `FinalApproachGate`：判断 final approach。
- `RateShaper`：低通、限幅、jerk 限制和 softcap。
- `SpeedGovernor`：高速缩放图像控制量。
- `EdgeProtection`：边缘保护和推力缩放。
- `PreclimbModule`：xy 稳定前限制推进/爬升。
- `ThrustManager`：等待、追踪、末端推力管理。
- `CommitModule`：末端快照和短时保持。
- `TiltGuard` / `DirectionalDive` / `TerminalPredictor`：姿态保护和末端预测/俯冲策略。

两者区别：

- `bf_flight_png`：PNG 前馈为主，状态少，结构清晰，适合对照论文和快速调试。
- `bf_flight`：状态机完整，参数更多，末端策略更复杂，适合阶段化控制和更强工程兜底。

## 14. 与论文的关系

`core/reference1.pdf` 的主题是基于图像视觉伺服的多旋翼精确拦截，核心思想是把 IBVS 与 PNG 结合，并通过视觉预测和视场保持降低图像延迟、末端过载和目标出框风险。

当前工程中的对应关系：

- `ex/ey`：图像平面角误差。
- `ex_dot/ey_dot`：LOS rate 的图像域近似。
- `bbox_area_ratio`：目标接近程度的工程近似。
- `nav_ratio_x/y`：PNG 导引比例。
- `visual_prediction_*`：检测延迟补偿。
- `fov_trim` / `edge_guard`：视场保持。
- `terminal_intercept` / `terminal_crossing`：末端预测和阻尼。
- `tilt_cap` / BF runtime gating：实机安全包络。

论文更偏导引律和实验验证；本仓库还包含 Betaflight 接管、MSP 输出、RC 映射、RKNN 推理、共享内存、Web 地面站和在线调参。

## 15. 上机前检查清单

编译：

- `cmake --build adapters/bf/build -j` 成功。
- `bf_flight_png`、`bf_debugd` 可执行。
- `ctest --test-dir adapters/bf/build --output-on-failure` 通过，或已记录不能运行的原因。

配置：

- `bf_flight_config: config/bf_flight_common.yaml` 指向有效文件。
- 相机 YAML、相机内参、RKNN 模型路径存在。
- MSP 串口设备和权限正确。
- `hover_thrust_z`、`strike_thrust_z` 与当前飞机油门标定一致。
- `dry_run` 阶段完成后才允许实控。

调试：

- 8080 页面可打开。
- `/api/video/status` 显示预览帧更新。
- `vision_ex/ey` 符号正确。
- `roll/pitch` 输出方向正确。
- `armed`、`msp_override_active`、watchdog 门控符合预期。
- 目标丢失、倾角超限和未解锁时都不会继续危险输出。
