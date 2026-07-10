# flutglove_dds

A minimal Foxglove-style ROS 2 viewer that pulls the live graph **directly over
CycloneDDS** — no bridge, no WebSocket, no ROS install. It is a second, simpler
frontend built on the same **cyclone_dds layer** ([`dds_direct`](../dds_direct) +
[`ros2_cdr`](../../dart/ros2_cdr)) that flutglove's `CycloneDDS` transport uses.

The point: grow **different project structures over the one DDS core**. flutglove
is the full configurable-panel app; `flutglove_dds` is the lean single-view one.

```
live ROS 2 graph ─DDS/RTPS→ CycloneDDS (dds_direct) ─→ ros2_cdr (Dart) ─→ this UI
                            └── discover DCPSPublication + decode with bundled .msg ──┘
```

## What it does

- **Discovers** every topic on the DDS graph (over `DCPSPublication`) and lists it
  with its ROS type — no ROS, no bridge.
- **Subscribes** to the selected topic over DDS and shows each decoded message as a
  live tree, decoded by `ros2_cdr` using the schema registry bundled in
  `dds_direct` (so any type decodes with nothing installed).
- Status bar: domain · topic count · msg/s.

## Run

```bash
flutter create .          # once, to generate platform runners
flutter run               # builds CycloneDDS (from dds_direct) and connects to domain 0
```

Point it at a robot by editing the `_domain` in `lib/main.dart` (or extend it with
a domain field). On the same LAN / host it discovers automatically.
