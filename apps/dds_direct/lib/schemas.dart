// schemas.dart — load the bundled ROS 2 .msg schema registry so the DDS
// transport can decode any discovered type without a ROS install.
import 'dart:convert';

import 'package:flutter/services.dart' show rootBundle;

/// Loads `assets/ros_schemas.json` (canonical `pkg/Type` -> concatenated
/// ros2msg text) shipped with this plugin. Pass the result to
/// `Ros2Dds.registerSchemas`.
Future<Map<String, String>> loadBundledSchemas() async {
  final raw = await rootBundle
      .loadString('packages/dds_direct/assets/ros_schemas.json');
  final map = jsonDecode(raw) as Map<String, Object?>;
  return map.map((k, v) => MapEntry(k, '$v'));
}
