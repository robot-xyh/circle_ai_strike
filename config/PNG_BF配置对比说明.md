# BF PNG 顶相机配置对比说明

对比对象（同一 `StrikePngController`，按目标接近/相对速度分档；算法参数对齐仿真 PNG 速度族）：

- `strike_png_bf_flight.yaml` — **默认基线**（BF 首飞保守混合，非纯速度档）
- `strike_png_bf_flight_low_speed.yaml` — 低速场景
- `strike_png_bf_flight_mid_speed.yaml` — 中速场景
- `strike_png_bf_flight_high_speed.yaml` — 高速场景

仿真对照：`circle_pilot_target_strike_png/config/gz_target_strike_png_top_cam_{low,mid,high}_speed.yaml`

> 核心设计思想与仿真一致：**目标越快 / 闭合越快 → 控制越激进、滤波越少、预测与前置量越大**。

启动示例：

```bash
./scripts/launch/launch_bf.sh --png --flight-config config/strike_png_bf_flight_low_speed.yaml
./scripts/launch/launch_bf.sh --png --flight-config config/strike_png_bf_flight_mid_speed.yaml
./scripts/launch/launch_bf.sh --png --flight-config config/strike_png_bf_flight_high_speed.yaml
```

---

## BF 与仿真差异（未随速度档变化）

| 项 | BF 三套速度档 | 说明 |
|---|---|---|
| `min_score` | 0.25 | 真机门限，仿真为 0.10 |
| `hover_thrust_z` / `strike_thrust_z` | 见 **[油门（thrust）参数](#油门thrust参数)** | BF 杆位分数，与 `strike_bf_flight` 对齐；**不可照抄**仿真 PX4 推力标度 |
| `derotate_body_rates` | false | BF 无机体角速度反馈 |
| `lateral/longitudinal_output_sign` | +1 / +1 | FLU vs FRD |
| `tilt_cap` | 全开 ±70° | 速度族均保留倾角包络 |
| `terminal_forward_speed_guard` | 开 | BF 通常无 NED 速度，实际可能不触发 |

## 油门（thrust）参数

PNG 在 BF 上通过 `strike_png.hover_thrust_z` / `strike_thrust_z` / `entry_smooth_*` 控制油门，含义与 PX4 仿真里的 **同名 key 不同物理量**：

| 平台 | 字段含义 | 典型悬停 | 典型打击 |
|------|----------|----------|----------|
| **仿真 / PX4** `target_strike_png.*` | PX4 归一化推力（`-1~0` 系或 0~1 推力标量，见节点实现） | `hover_thrust_z` **0.58** | 随速度档 **0.80 / 0.76 / 0.83** |
| **BF** `strike_png.*` | Betaflight **杆位分数** `[0,1]`：`0→1000 PWM`，`0.5→1500`，`1.0→2000`（`PWM ≈ 1000 + scalar×1000`） | **0.283**（≈1283 PWM） | **0.50**（1500 PWM） |
| **BF Strike PD** `strike.thrust.*` | 同上杆位分数 | `hover_scalar` **0.283** | `constant_scalar` **0.50** |

BF PNG 速度族已与 `strike_bf_flight.yaml` 油门带对齐，**不要**把仿真 `strike_thrust_z` 0.76~0.83 数值抄进 BF YAML。

### 全量对照（当前仓库配置）

| 配置 | `hover_thrust_z` | `strike_thrust_z` | `entry_smooth_initial_thrust_z` | `entry_smooth_duration_s` |
|------|------------------|-------------------|----------------------------------|---------------------------|
| BF `strike_png_bf_flight_low_speed` | 0.283 | 0.50 | 0.283 | 0.80 |
| BF `strike_png_bf_flight_mid_speed` | 0.283 | 0.50 | 0.283 | 0.80 |
| BF `strike_png_bf_flight_high_speed` | 0.283 | 0.50 | 0.283 | 0.60 |
| BF `strike_png_bf_flight`（默认基线） | 0.078 | 0.082 | 0.078 | 0.80 |
| 仿真 `gz_*_low_speed` | 0.58 | 0.80 | 0.58 | 0.80 |
| 仿真 `gz_*_mid_speed` | 0.58 | 0.76 | 0.58 | 0.80 |
| 仿真 `gz_*_high_speed` | 0.58 | 0.83 | 0.58 | 0.60 |
| 仿真 `gz_*_top_cam`（基线） | 0.58 | 0.80 | 0.58 | 0.80 |

### 速度分档与油门

- **BF 三套速度档**：`hover` / `strike` / `entry_smooth_initial` **完全相同**（0.283 / 0.50 / 0.283）；仅 `entry_smooth_duration_s` 在高速档缩短为 **0.60**（与仿真 high 一致），用于更快切入算法油门。
- **仿真 PNG 三套**：`hover` 恒 **0.58**；`strike_thrust_z` 按档变化（0.80 / 0.76 / 0.83），用**打击推力标量**表达「越快闭合越猛」。
- **BF 等效做法**：闭合力度不靠分档 `strike_thrust_z`，而靠 **`closure_base_scale` / `closure_area_gain`** 及末端 PN/拦截项（见下一节参数表）。

### 调参提示

1. **台架先标 hover**：确认 `hover_thrust_z=0.283` 能稳定悬停（与 `strike_bf_flight` 的 `hover_scalar` 一致）；不对则只改 hover，再改 strike。
2. **打击档**：`strike_thrust_z=0.50` 为 Tracking 目标油门（≈1500 PWM）；过大易抬头冲过，过小追不上——优先动 **`closure_*`**，再微调 `strike_thrust_z`（三档可同改）。
3. **入场平滑**：`entry_smooth_initial_thrust_z` 应 **≈ hover**（当前速度族均为 0.283），避免接管瞬间油门台阶。
4. **默认基线** `strike_png_bf_flight.yaml` 仍保留旧首飞档 **0.078 / 0.082**；正式打靶请用 `*_low/mid/high_speed.yaml`，或把基线三项改为 0.283 / 0.50 / 0.283。
5. **末端抬头过猛**：降 `strike_thrust_z` 或 `closure_*`；BF 无 NED 速度时 `terminal_forward_speed_guard` 往往不生效。

---

## 一、随速度档单调变化的参数

| 参数 | low | mid | high | 含义 |
|---|---|---|---|---|
| `max_roll_rate_rad_s` | 1.2 | 1.35 | 1.45 | 角速度输出上限 |
| `max_pitch_rate_rad_s` | 1.45 | 1.2 | 1.70 | 同上 |
| `nav_ratio_x` | 3.0 | 3.0 | 3.4 | PN 横向增益 |
| `nav_ratio_y` | 2.2 | 2.0 | 2.8 | PN 纵向增益 |
| `closure_base_scale` | 0.65 | 0.7 | 0.85 | 闭合基准 |
| `closure_area_gain` | 0.30 | 0.35 | 0.45 | 面积→闭合增益 |
| `max_feedforward_rad_s` | 0.90 | 1.05 | 1.10 | 前馈上限 |
| `pixel_dot_lpf_tau_s` | 0.08 | 0.08 | 0.055 | 像速 LPF（高速更小） |
| `los_rate_hold_tau_s` | 0.08 | 0.08 | 0.06 | 视线速率保持 |
| `visual_prediction_max_age_s` | 0.18 | 0.22 | 0.25 | 外推时长 |
| `visual_prediction_max_offset_norm` | 0.12 | 0.16 | 0.20 | 外推位移上限 |
| `terminal_intercept_lead_s` | 0.16 | 0.20 | 0.26 | 末端拦截前置 |
| `terminal_intercept_kp_rate` | 1.6 | 1.45 | 1.9 | 末端拦截增益 |
| `terminal_intercept_max_rate_rad_s` | 0.26 | 0.28 | 0.32 | 末端拦截角速度上限 |
| `terminal_crossing_kd_rate` | 0.80 | 0.70 | 0.95 | 横穿阻尼 |
| `terminal_crossing_max_rate_rad_s` | 0.24 | 0.18 | 0.28 | 横穿修正上限 |
| `detection_stale_s` | 0.35 | 0.35 | 0.30 | 检测过期判定 |
| `entry_smooth_duration_s` | 0.80 | 0.80 | 0.60 | 入场平滑时长 |
| `target_lost_hold_delay_s` | 0.05 | 0.05 | 0.04 | 丢失后释放物理保持（s） |
| `fov_trim_kp_rate` | 0.22 | 0.35 | 0.35 | 视场修正增益 |
| `fov_trim_fade_area_ratio_start/full` | 1.0/1.0 | 0.002/0.008 | 0.002/0.008 | 末端 fov_trim 淡出 |
| `vertical_aim_ey` | 0.04 | 0.02 | 0.02 | 垂直瞄准偏置 |

---

## 二、结构性差异

| 项 | low | mid | high |
|---|---|---|---|
| `terminal_tilt_aim` 占位块 | gain=0（关） | gain=0（占位，与仿真 mid 一致） | gain=0（关） |
| `fov_trim_fade` | 关（1.0/1.0） | 开 | 开 |
| 末端 stale-trim 阈值 | 标准 | 标准 | 前移 + 增益加大 |

---

## 三、默认基线 `strike_png_bf_flight.yaml` 说明

默认文件保留 BF 首飞混合参数（如 `nav_ratio` 2.5、`max_*_rate` 2.0、`pixel_dot_lpf` 0.055），**不等同于任一套速度档**。油门仍为旧保守档（见 [油门（thrust）参数](#油门thrust参数) 对照表）；标定后请改用 `*_low/mid/high_speed.yaml`。

现场按目标速度选用 `*_low/mid/high_speed.yaml` 即可与仿真 PNG **控制增益**分档对齐；**油门标量**三档 BF 相同，与仿真「分档 strike_thrust」策略不同，闭合差异见 `closure_*` 与末端项。
