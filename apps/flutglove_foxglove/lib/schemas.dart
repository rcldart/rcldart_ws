import 'dart:convert';
import 'package:flutter/services.dart' show rootBundle;
Future<Map<String, String>> loadBundledSchemas() async {
  final raw = await rootBundle.loadString('assets/ros_schemas.json');
  final map = jsonDecode(raw) as Map<String, Object?>;
  return map.map((k, v) => MapEntry(k, '$v'));
}
