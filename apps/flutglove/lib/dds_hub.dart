// dds_hub.dart — a flutglove TopicSource that pulls the live ROS 2 graph
// DIRECTLY over CycloneDDS (via the dds_direct plugin), with NO bridge and NO
// WebSocket. Discovery + decode are pure DDS + pure-Dart CDR; message schemas
// come from the bundled registry so any discovered type decodes with no ROS.
import 'dart:async';

import 'package:dds_direct/dds_direct.dart';
import 'package:dds_direct/schemas.dart';

import 'topic_source.dart';

class DdsDirectHub implements TopicSource {
  DdsDirectHub._(this._node);

  final Ros2Dds _node;
  Map<String, String> _graph = {}; // '/topic' -> 'pkg/msg/Type'
  final Map<String, StreamSubscription<void>> _subs = {};

  /// Open a participant on [domain], load the bundled schema registry, and do a
  /// first graph read. Talks straight to the DDS graph — no bridge.
  static Future<DdsDirectHub> create({int domain = 0}) async {
    final node = Ros2Dds(domain: domain);
    node.registerSchemas(await loadBundledSchemas());
    final hub = DdsDirectHub._(node);
    hub.refreshGraph();
    return hub;
  }

  @override
  Iterable<String> get topics => _graph.keys;

  @override
  int get topicCount => _graph.length;

  @override
  String? typeOf(String topic) => _graph[topic];

  @override
  void subscribe(String topic, TopicListener cb) {
    final type = _graph[topic];
    if (type == null) return;
    final ddsTopic = topic.startsWith('/') ? topic.substring(1) : topic;
    _subs['$topic ${cb.hashCode}'] = _node.subscribe(ddsTopic, type, cb);
  }

  @override
  void unsubscribe(String topic, TopicListener cb) {
    _subs.remove('$topic ${cb.hashCode}')?.cancel();
  }

  @override
  void refreshGraph() => _graph = _node.discover();

  @override
  void spinOnce() {/* subscriptions poll on their own timers */}

  @override
  void dispose() {
    for (final s in _subs.values) {
      s.cancel();
    }
    _subs.clear();
    _node.dispose();
  }
}
