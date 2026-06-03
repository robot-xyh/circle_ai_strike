# circle_ai_strike

`circle_ai_strike` is the first-phase extraction of the Betaflight real-aircraft PNG/PN strike stack from `pilot_ws`. This project intentionally keeps the original `src/circle_pilot/core` and `src/circle_pilot/adapters` layout, package names, target names, and include paths so changes can be compared against the original workspace with minimal noise.

## Current scope

Included:

- Betaflight real-aircraft launch script: `src/circle_pilot/scripts/launch/launch_bf.sh`.
- Betaflight PNG binary stack: `bf_flight_png`, `bf_debugd`, `bf_runtime`, `bf/common`.
- PNG/PN control core under `src/circle_pilot/core`.
- BF shared runtime support under `src/circle_pilot/adapters/common/{ipc,debug,perception}`.
- Real-device camera and inference assets needed by `config/bf_flight_common.yaml`:
  - camera configs in `src/circle_pilot/config/`;
  - camera intrinsics in `src/circle_pilot/circle_pilot_bringup/config/`;
  - RKNN model assets and the corresponding YOLO `.pt` source model in `src/circle_pilot/circle_pilot_object_tracking/models/`.

Not included in this phase:

- PX4 SITL, Gazebo worlds/models, ROS2 launch packages, DDS agent, ROS2 YOLO/debug/bringup packages.
- Full `circle_pilot_bringup` or `circle_pilot_object_tracking` packages; only the asset subdirectories required by BF real-aircraft runtime are present.

## Layout

```text
src/circle_pilot/
├── adapters/
│   ├── bf/                 # BF binaries and runtime CMake project
│   └── common/
│       ├── debug/          # bf_debugd overlay/video/debug support
│       ├── ipc/            # shared-memory telemetry/params/preview contracts
│       └── perception/     # V4L2 camera, MPP/RGA, RKNN, zero-copy pipeline
├── circle_pilot_bringup/
│   └── config/             # camera intrinsics only
├── circle_pilot_object_tracking/
│   └── models/             # RKNN model assets only
├── config/                 # BF flight/debug/camera configs
├── core/                   # PNG/PN control core and shared strike types
└── scripts/launch/         # BF launch scripts only
```

## Build

Build the Betaflight real-aircraft binaries from the BF adapter CMake project:

```bash
cmake -S src/circle_pilot/adapters/bf -B src/circle_pilot/adapters/bf/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build src/circle_pilot/adapters/bf/build -j
```

Expected binaries:

```text
src/circle_pilot/adapters/bf/build/bf_flight_png
src/circle_pilot/adapters/bf/build/bf_debugd
```

## Run

From the project root, the BF launcher can be checked without touching hardware:

```bash
SKIP_PERFORMANCE=1 src/circle_pilot/scripts/launch/launch_bf.sh --help
```

On the target aircraft, after verifying MSP device, camera device, RKNN runtime, MPP/RGA libraries, and config values, run:

```bash
src/circle_pilot/scripts/launch/launch_bf.sh --png
```

The launcher defaults to:

```text
src/circle_pilot/adapters/bf/build/bf_flight_png
src/circle_pilot/adapters/bf/build/bf_debugd
src/circle_pilot/config/strike_png_bf_flight.yaml
src/circle_pilot/config/strike_png_bf_debug.yaml
```

## Runtime asset paths

The active BF shared config is:

```text
src/circle_pilot/config/bf_flight_common.yaml
```

Important runtime references are project-local relative paths:

```yaml
camera: config/camera_bf_mono_011_lowres.yaml
camera_info: circle_pilot_bringup/config/mono_a1_011_top_6mm_640x512.yaml
model_path: circle_pilot_object_tracking/models/drone_v8n_v21_kd_relu_lambda008_640_640_rknn_model/drone_v8n_v21_kd_relu_lambda008_640_640-rk3588.rknn
```

## Comparison policy

This repository should stay close to the original `pilot_ws/src/circle_pilot` source structure. Prefer copying upstream changes into the same relative paths instead of renaming packages or refactoring directories. Generated build directories, logs, debug captures, and compile databases are ignored by `.gitignore`.
