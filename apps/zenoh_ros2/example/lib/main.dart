// Demo of zenoh_ros2: reach a ROS 2 graph over Zenoh with NO ROS installed.
// On the robot/host run `zenoh-bridge-ros2dds` (or rmw_zenoh + rmw_zenohd), then
// point --connect at it. Run: `flutter create .` here once, then `flutter run
// --dart-define=CONNECT=tcp/<robot-ip>:7447`.
import 'dart:async';

import 'package:flutter/material.dart';
import 'package:zenoh_ros2/zenoh_ros2.dart';

void main() => runApp(const DemoApp());

class DemoApp extends StatefulWidget {
  const DemoApp({super.key});
  @override
  State<DemoApp> createState() => _DemoAppState();
}

class _DemoAppState extends State<DemoApp> {
  final _log = <String>[];
  Ros2Zenoh? _node;

  @override
  void initState() {
    super.initState();
    _start();
  }

  Future<void> _start() async {
    const connect = String.fromEnvironment('CONNECT');
    try {
      final node = await Ros2Zenoh.open(
          connect: connect.isEmpty ? const [] : [connect])
        ..registerType('std_msgs/msg/String', 'string data');
      await node.subscribe('/chatter', 'std_msgs/msg/String', (msg) {
        setState(() => _log.insert(0, 'heard: ${msg['data']}'));
      });
      final pub = await node.advertise('/chatter', 'std_msgs/msg/String');
      var i = 0;
      Timer.periodic(const Duration(seconds: 1),
          (_) => pub.publish({'data': 'Hello ROS 2 from Dart/Zenoh: ${i++}'}));
      _node = node;
    } catch (e) {
      setState(() => _log.insert(0, 'init error: $e'));
    }
  }

  @override
  void dispose() {
    _node?.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('zenoh_ros2 — ROS 2 over Zenoh, no ROS')),
        body: ListView(children: [for (final l in _log) ListTile(title: Text(l))]),
      ),
    );
  }
}
