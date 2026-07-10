// Minimal demo of the dds_direct FFI plugin: publishes and subscribes to
// std_msgs/String on /chatter over DDS — NO ROS installed on the device.
//
// Run: `flutter create .` here once (to generate platform runners), then
// `flutter run`. The plugin builds CycloneDDS from source automatically.
import 'dart:async';

import 'package:dds_direct/dds_direct.dart';
import 'package:flutter/material.dart';

void main() => runApp(const DemoApp());

class DemoApp extends StatefulWidget {
  const DemoApp({super.key});
  @override
  State<DemoApp> createState() => _DemoAppState();
}

class _DemoAppState extends State<DemoApp> {
  final _log = <String>[];
  Ros2Dds? _node;
  Ros2Publisher? _pub;
  int _i = 0;
  Timer? _timer;

  @override
  void initState() {
    super.initState();
    _start();
  }

  Future<void> _start() async {
    try {
      final node = Ros2Dds(domain: 0)
        ..registerType('std_msgs/msg/String', 'string data');
      node.subscribe('chatter', 'std_msgs/msg/String', (msg) {
        setState(() => _log.insert(0, 'heard: ${msg['data']}'));
      });
      _pub = node.advertise('chatter', 'std_msgs/msg/String');
      _timer = Timer.periodic(const Duration(seconds: 1), (_) {
        _pub?.publish({'data': 'Hello ROS 2 from Dart: ${_i++}'});
      });
      _node = node;
    } catch (e) {
      setState(() => _log.insert(0, 'init error: $e'));
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    _node?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('dds_direct — ROS 2, no ROS install')),
        body: ListView(children: [for (final l in _log) ListTile(title: Text(l))]),
      ),
    );
  }
}
