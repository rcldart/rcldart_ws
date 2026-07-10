# flutglove_foxglove

A minimal ROS 2 subscribe+publish frontend over the **Foxglove WebSocket bridge**
— same tabbed architecture as `flutglove_dds`. Discovery and per-message schemas
come from the bridge (so subscribe/decode needs nothing bundled); publish
serializes JSON with [`ros2_cdr`](../../dart/ros2_cdr) and uses the bridge's
`clientPublish` capability. One outbound TCP, crosses NAT, no ROS install.

Robot side: `ros2 run foxglove_bridge foxglove_bridge`. App side: connect to
`ws://<host>:8765`, then Subscribe (browse the discovered graph) / Publish.

```bash
flutter create .
flutter run
```
