# rcldart_ws

A ROS 2 **workspace + Dart bridge + apps** for [rcldart](https://github.com/harunkurtdev/rcldart) —
author custom interface packages (`.msg` / `.srv` / `.action`), turn them into
Dart packages, and consume them from Flutter apps that talk to live ROS 2.

```
rcldart_ws/
├── src/            colcon interface packages  (rcldart_msgs, …)       ← you author here
├── dart/           generated Dart packages     (std_msgs, sensor_msgs, …)  ← auto-generated
├── apps/           standalone Flutter apps      (flutglove, ros2_viz)  ← consume dart/ + rcldart
└── tool/           sync_dart_packages.sh + sync_app_overrides.py       ← the automation
```

## The pipeline (src → dart → apps)

```
src/<pkg>/msg/*.msg          author a message / service / action
    │  colcon build          → install/<pkg>/share/**/*.idl
    ▼
rosidl_generator_dart        → dart/<pkg>/  (pubspec + Dart classes, deps resolved)
    │
    ▼
apps/<app>/pubspec.yaml      dependency_overrides: <pkg>: { path: ../../dart/<pkg> }
    │  flutter pub get
    ▼
import 'package:<pkg>/<pkg>.dart';   use the generated types with rcldart
```

## Make custom packages usable by apps — automatically

One command does all three steps (build → generate → wire into every app):

```bash
tool/sync_dart_packages.sh              # all packages in src/, all apps
tool/sync_dart_packages.sh my_msgs      # just one package
```

It runs `colcon build`, then `gen_ros_dart_ws.py` (resolving dependencies), then
`sync_app_overrides.py` which **injects the `dependency_overrides` path entries
into every app's `pubspec.yaml`** (idempotent — existing entries are left alone,
both two-line and inline `{ path: … }` forms are recognized). Afterwards just run
`flutter pub get` in the app.

See [docs/custom_packages.md](docs/custom_packages.md) for a full worked example
(`rcldart_msgs`: `RobotStatus.msg` + `GetDistance.srv`).

## Apps

- **flutglove** — a Foxglove-like configurable-panel viewer. Decodes **any** live
  topic at runtime (via `ros_cdr` introspection, no per-type codegen), mosaic
  split panels, Raw tree / Plot / Viz (Laser, OccupancyGrid, Image/CompressedImage),
  searchable topic sidebar, save/load layouts. Linux desktop today; Android/iOS/
  macOS infrastructure in place (see `../docs/`).
- **ros2_viz** — a multi-tab dashboard + teleop/route control panel reading live
  `/scan`, `/odom`, `/tf`, `/battery_state`, costmaps, diagnostics.

Run an app (source ROS so it interops with a running stack's RMW):

```bash
cd apps/flutglove
flutter pub get && flutter build linux
source /opt/ros/$ROS_DISTRO/setup.bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./build/linux/x64/debug/bundle/flutglove
```

## Requirements

- ROS 2 Jazzy — for `colcon build` of `src/` and, on desktop, the runtime.
- Flutter — Linux desktop toolchain (Android/Apple per the rcldart docs).
- The generated Dart packages are pure Dart + FFI and are platform-independent.

## What is / isn't committed

- **Committed**: `src/` (your interfaces), `dart/` generated packages (so the
  apps work straight after clone), `apps/` sources, `tool/`, `docs/`.
- **Ignored** (`.gitignore`): `build/`, `install/`, `log/`, all `.dart_tool/`
  and Flutter/`build/` output, and the large per-platform bundled ROS closures.

`dart/` is machine-generated — regenerate any time with
`tool/sync_dart_packages.sh`.
