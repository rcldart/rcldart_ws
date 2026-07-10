# zenoh_ros2 — ROS 2 over Zenoh + Dart CDR (via zenoh-bridge-ros2dds)

A dependency-minimal ROS 2 client that reaches a ROS 2 graph through **Zenoh** —
the lightest possible native footprint (one Rust lib, `zenoh-c`, already vendored
as [`zenoh_ffi`](../../../zenoh_ffi) and built by cargo/FetchContent) — and does
all message serialization in **pure Dart** ([`ros2_cdr`](../../dart/ros2_cdr)).

## Why this exists / how it differs from `dds_direct`

Two dependency-minimal routes to ROS 2 without a ROS install:

| | `dds_direct` | **`zenoh_ros2`** |
|--|--|--|
| Native dep | CycloneDDS (`libddsc`) | zenoh-c (1 Rust lib) — lightest |
| Transport | direct DDS/RTPS P2P, **no bridge** | Zenoh; needs a bridge for DDS graphs |
| Robot side | nothing | run `zenoh-bridge-ros2dds` **or** use `rmw_zenoh` |
| NAT / mobile | multicast-ish | **one outbound TCP** — NAT-friendly |
| Serialization | pure-Dart `ros2_cdr` | pure-Dart `ros2_cdr` (same) |

The bridge (`zenoh-bridge-ros2dds`) discovers the DDS/ROS graph and re-exposes
every topic over Zenoh; the payload it forwards **is** the raw CDR, so this app
just declares a Zenoh subscriber/publisher on the mapped key and de/encodes the
CDR in Dart. If the robot runs `rmw_zenoh`, no bridge is needed at all.

```
ROS 2 (DDS)  ⇄  zenoh-bridge-ros2dds  ⇄  Zenoh (1 TCP)  ⇄  zenoh_ffi  ⇄  ros2_cdr (Dart)
                (on the robot/host)                        └── zero ROS on this side ──┘
```

## Layout

| Path | What |
|------|------|
| `lib/zenoh_ros2.dart` | `Ros2Zenoh` — open/advertise/subscribe with Dart maps |
| `lib/src/key_mapping.dart` | ROS topic ⇄ Zenoh key (bridge-ros2dds default; configurable) |
| `bin/listener.dart` / `bin/talker.dart` | `std_msgs/String` demo |

## Status

- ✅ Serialization (`ros2_cdr`): encode+decode, unit-tested.
- ✅ Zenoh transport via the vendored `zenoh_ffi` (cargo/FetchContent, no ROS).
- ✅ Pub/sub wiring + CDR de/encode of the Zenoh payload.
- ⏳ **Key mapping**: defaults to the `zenoh-bridge-ros2dds` convention (topic
  without the leading `/`). `rmw_zenoh` uses a mangled `<domain>/<topic>/<type>/
  <hash>` keyexpr — pass a custom mapper for that. Verify against your bridge.

## Run

On the robot / host (has ROS):
```bash
# either the DDS↔Zenoh bridge …
zenoh-bridge-ros2dds
# … or, if the graph already runs rmw_zenoh, just a router:
ros2 run rmw_zenoh_cpp rmw_zenohd
```
On this side (no ROS) — it's a library; use it from any Flutter/Dart app:
```yaml
dependencies:
  zenoh_ros2: ^0.1.0   # pulls the zenoh_ffi FFI plugin (native builds itself)
```
Or run the bundled demo:
```bash
cd example
flutter create .   # once, to generate platform runners
flutter run --dart-define=CONNECT=tcp/<robot-ip>:7447
```
