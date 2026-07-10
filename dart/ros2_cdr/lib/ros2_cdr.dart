/// Pure-Dart ROS 2 CDR codec + `.msg` schema parser — the serialization half of
/// a dependency-minimal ROS 2 client (no rosidl typesupport, no ROS install).
///
/// ```dart
/// final reg = MsgRegistry()
///   ..addConcatenated('std_msgs/msg/String', 'string data');
/// final codec = Ros2Codec(reg);
/// final bytes = codec.encode('std_msgs/msg/String', {'data': 'hi'});
/// final back  = codec.decode('std_msgs/msg/String', bytes); // {data: hi}
/// ```
library;

export 'src/cdr.dart';
export 'src/codec.dart';
export 'src/msg_def.dart';
