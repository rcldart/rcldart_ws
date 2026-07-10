# flutglove_zenoh

A minimal ROS 2 subscribe+publish frontend over **Zenoh** — same tabbed
architecture as `flutglove_dds`, but the transport layer is
[`zenoh_ros2`](../zenoh_ros2) (Zenoh via a `zenoh-bridge-ros2dds`) with pure-Dart
[`ros2_cdr`](../../dart/ros2_cdr) serialization. One outbound TCP, NAT-friendly,
no ROS install.

Robot side: `zenoh-bridge-ros2dds` (or `rmw_zenoh`). App side: enter the router
endpoint, then Subscribe (topic+type) / Publish (topic+type+JSON).

```bash
flutter create .
flutter run
```
