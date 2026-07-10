// schemas.dart — load the bundled ROS 2 .msg schema registry so ros2_cdr can
// decode/encode any type over Zenoh with no ROS install.
import 'dart:convert';

import 'package:flutter/services.dart' show rootBundle;

Future<Map<String, String>> loadBundledSchemas() async {
  final raw = await rootBundle.loadString('assets/ros_schemas.json');
  final map = jsonDecode(raw) as Map<String, Object?>;
  return map.map((k, v) => MapEntry(k, '$v'));
}
