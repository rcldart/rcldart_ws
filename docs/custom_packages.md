# Adding a custom package (msg / srv / action) and using it in an app

This walks through creating a custom interface package under `src/`, turning it
into a Dart package, and having it show up in the apps — **automatically**.

## TL;DR

```bash
# 1. author src/my_msgs/{msg,srv}/*.{msg,srv}  (+ package.xml, CMakeLists.txt)
# 2. one command:
tool/sync_dart_packages.sh my_msgs
# 3. in the app:
cd apps/flutglove && flutter pub get
#    import 'package:my_msgs/my_msgs.dart';
```

## 1. Author the interface package

A colcon interface package is just `package.xml` + `CMakeLists.txt` +
`msg/`/`srv/`/`action/` files. Minimal example `src/my_msgs/`:

`src/my_msgs/msg/RobotStatus.msg`
```
std_msgs/Header header
string name
uint8 mode
bool healthy
float64[] battery_cells
```

`src/my_msgs/srv/GetDistance.srv`
```
geometry_msgs/Point target
---
float64 distance
```

`src/my_msgs/package.xml`
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_msgs</name>
  <version>0.0.1</version>
  <description>Custom interfaces</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

`src/my_msgs/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/GetDistance.srv"
  DEPENDENCIES std_msgs geometry_msgs)
ament_package()
```

## 2. Sync (build → generate → wire into apps)

```bash
tool/sync_dart_packages.sh my_msgs
```

What happens:
1. **`colcon build --packages-select my_msgs`** → `install/my_msgs/share/**/*.idl`
   + `libmy_msgs__rosidl_typesupport_c.so`.
2. **`gen_ros_dart_ws.py dart my_msgs`** → `dart/my_msgs/` (and any missing
   dependency packages like `std_msgs`, `geometry_msgs` — deps are resolved by
   scanning the `.idl` refs and generated in topological order).
3. **`sync_app_overrides.py`** → adds to every `apps/*/pubspec.yaml`:
   ```yaml
   dependency_overrides:
     my_msgs:
       path: ../../dart/my_msgs
   ```
   (idempotent — nothing is duplicated if it's already there).

## 3. Use it in an app

```bash
cd apps/flutglove
flutter pub get
```

```dart
import 'package:my_msgs/my_msgs.dart';
import 'package:rcldart/rcldart.dart';

final node = RclDart().createNode('demo', 'demo');
final pub = node.createPublisher(
  topic_name: '/robot_status',
  messageType: MyMsgsRobotStatus(),   // generated wrapper (CamelCase pkg+type)
);
```

The wrapper class name is `<Pkg><Type>` in CamelCase — `my_msgs/msg/RobotStatus`
→ `MyMsgsRobotStatus`. Nested fields follow the FFI struct (`data.ref.…`);
strings and sequences have Dart getters/setters. Any topic can also be read with
**no** generated class at all via `DynamicTopicHub` (runtime introspection) — see
flutglove.

## Is it "automatic"?

- **Generation + app wiring: yes** — `tool/sync_dart_packages.sh` builds,
  generates, and injects the pubspec overrides for every app in one command.
- **You still author the `.msg`/`.srv`** and run the sync (there is no file
  watcher). Re-run the sync whenever you change interfaces.
- **Runtime**: on desktop, `source install/setup.bash` (or the ROS env) so
  `libmy_msgs__rosidl_typesupport_c.so` is found. For no-ROS deployments bundle
  it — Linux `../tool/bundle_ros_libs.sh`, Android/Apple via the platform build
  scripts (the packagers include your package's typesupport + introspection libs).

## Notes / gotchas

- **Override-only packages resolve fine** in pub — an app can carry a
  `dependency_overrides` entry it doesn't `import`; it just isn't used.
- The generated `std_msgs` **replaces** any hand-written one for the app (it
  imports `builtin_interfaces` instead of redefining `Time`).
- `dart:ffi` in a Flutter file needs `import 'dart:ffi' hide Size;` (clashes with
  `dart:ui.Size`).
- Regenerate all packages at once with `tool/sync_dart_packages.sh` (no args).
