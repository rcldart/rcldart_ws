# dds_direct — bridgeless ROS 2 over CycloneDDS-FFI + Dart CDR

A **Flutter FFI plugin** (pub.dev-installable, like `zenoh_ffi`) that talks
**directly** to a ROS 2 graph over DDS/RTPS — **no ROS install, no rosidl
typesupport, no bridge process**. Add it as a dependency and the native DDS layer
(CycloneDDS, fetched & built from source by the plugin's own platform build)
compiles and bundles automatically. All message serialization is **pure Dart**
([`ros2_cdr`](../../dart/ros2_cdr)).

```yaml
# consumer app pubspec.yaml — that's all; native builds itself
dependencies:
  dds_direct: ^0.1.0
```

## Why this exists

The full rcldart Android closure is **181 native libs** — but only **one**
(`libddsc`) is the actual DDS wire protocol. The other 180 are ROS's per-type
`rosidl` typesupport `.so`s + `rcl`/`rmw`/`ament` machinery, and they are the
**only** reason the build needs a host ROS (for `rosidl` code generation).

`dds_direct` removes that entire half:

```
ROS 2 node  ⇄  RTPS/DDS wire  ⇄  libddsc (FetchContent)  ⇄  Dart FFI  ⇄  ros2_cdr (pure Dart)
                                 └──────── ~1 native lib, ZERO ROS ────────┘
```

Precedent: `ros2-client`+`rustdds` (Rust) and the `cyclonedds` Python bindings
already interoperate with ROS 2 with **no ROS installed** — same idea, in Dart.

## Layout (Flutter FFI plugin, mirrors `zenoh_ffi`)

| Path | What |
|------|------|
| `src/CMakeLists.txt` | `FetchContent` CycloneDDS + builds `libdds_direct` (shared C shim) |
| `src/ddsros.c/.h` | thin C shim: participant + raw-CDR writer/reader over a ROS-compatible topic |
| `linux/` · `windows/` · `android/build.gradle` | per-platform native build (delegate to `src/`) |
| `macos/` · `ios/` (podspec + `Classes/`) | CocoaPods build of the same C via CMake |
| `lib/src/ddsros_ffi.dart` | Dart FFI bindings (opens the bundled plugin lib) |
| `lib/dds_direct.dart` | `Ros2Dds` — Node-like API: `advertise` / `subscribe` (Dart maps in/out) |
| `example/` | Flutter demo app (talker + listener) |

## Status

- ✅ **Serialization** (`ros2_cdr`): encode+decode, schema-driven, unit-tested.
- ✅ **CMake FetchContent** of CycloneDDS (no ROS needed to build).
- ✅ **C shim skeleton** + Dart FFI: participant / writer / reader / raw-CDR I/O.
- ⏳ **ROS-node interop (the hard part):** CycloneDDS needs a `sertype` that
  carries the raw CDR blob and advertises the **ROS type name**
  (`std_msgs::msg::dds_::String_`) + topic name (`rt/<topic>`). The shim mirrors
  `rmw_cyclonedds`'s `sertype_rmw`. Remaining: ROS 2 **type hash** (RIHS01 /
  XTypes) for clean discovery matching on Humble/Jazzy, and QoS-profile matching.
  Until then app↔app pub/sub works; matching real ROS nodes is the next step.

## Build & run

As a plugin the native build runs automatically for the consumer app. To try the
bundled demo (it builds CycloneDDS from source the first time — needs cmake + a C
compiler, **no ROS**):

```bash
cd example
flutter create .          # once, to generate platform runners
flutter run               # builds CycloneDDS + the shim, then runs talker/listener
```

Talks to any ROS 2 node on the same DDS domain — e.g. `ros2 run demo_nodes_cpp
listener` will hear the demo's `/chatter`, once the type-hash TODO below lands.
