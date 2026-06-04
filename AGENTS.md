# Repository Guidelines

## Project Structure & Module Organization

This repository preserves the original `circle_pilot` layout while keeping directories at the repo root. Core guidance and control logic live in `core/`, with public headers under `core/include/circle/`, implementations under `core/src/`, and focused smoke tests under `core/tests/`. Betaflight integration lives in `adapters/bf/`, including `bf_flight`, `bf_flight_png`, `bf_debugd`, `bf_runtime`, shared BF helpers, and adapter tests. Shared runtime services are in `adapters/common/{ipc,debug,perception}`. Runtime YAML files are in `config/`, launch helpers are in `scripts/launch/`, camera intrinsics are in `circle_pilot_bringup/config/`, and RKNN/model assets are in `circle_pilot_object_tracking/models/`.

## Build, Test, and Development Commands

- `cmake -S adapters/bf -B adapters/bf/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`: configure the full BF adapter build.
- `cmake --build adapters/bf/build -j`: build `bf_flight`, `bf_flight_png`, and `bf_debugd`.
- `cmake -S adapters/bf -B adapters/bf/build -DCIRCLE_BF_BUILD_TESTS=ON -DCIRCLE_STRIKE_BUILD_TESTS=ON`: configure adapter and core tests.
- `ctest --test-dir adapters/bf/build --output-on-failure`: run configured CTest tests.
- `SKIP_PERFORMANCE=1 scripts/launch/launch_bf.sh --help`: validate launcher wiring without touching hardware.

For core-only work, use `cmake -S core -B core/build -DCIRCLE_STRIKE_BUILD_TESTS=ON`, then build and run `ctest --test-dir core/build --output-on-failure`.

## Coding Style & Naming Conventions

Use C++17 and keep code warning-clean under `-Wall -Wextra -Wpedantic`. Follow the existing 2-space indentation style. Public APIs belong in `include/circle/...`; implementations belong in matching `src/...` paths. Use `snake_case` filenames, `circle::...` namespaces, `PascalCase` types, lower-camel function names, and `snake_case` struct fields. Prefer small, explicit modules over broad refactors because this tree is meant to stay comparable with the upstream `pilot_ws/src/circle_pilot` layout.

## Testing Guidelines

Tests are CMake/CTest executables using `assert` or local `CHECK` macros, not a heavyweight test framework. Name new tests `*_test.cpp`, keep them hardware-free when possible, and add them to the nearest `CMakeLists.txt`. Cover pure control logic, YAML parsing, adapter mapping, and safety gating before changing runtime behavior.

## Commit & Pull Request Guidelines

Git history currently only shows an initial import, so use concise imperative commits such as `fix png adapter gating` or `add bf runtime smoke test`. Pull requests should describe behavior changes, list build/test commands run, mention config or model path changes, and include screenshots or logs only for UI/debug-output changes.

## Security & Configuration Tips

Do not commit generated build trees, logs, debug captures, or local IDE files. Keep deployment-specific device paths and credentials out of tracked YAML; start from `config/deployment.example.yaml` when adding local configuration.
