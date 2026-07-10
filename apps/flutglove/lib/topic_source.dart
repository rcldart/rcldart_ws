// topic_source.dart
//
// The single surface every flutglove panel consumes, so the transport
// underneath (native rcl DDS, or the Foxglove WebSocket bridge) is swappable
// without touching any panel. Both [RclHub] (wraps rcldart's DynamicTopicHub)
// and [FoxgloveWsHub] implement it.
import 'package:rcldart/rcldart.dart' as ros;

typedef TopicListener = void Function(Map<String, Object?> message);

abstract class TopicSource {
  /// Currently-known topic names.
  Iterable<String> get topics;

  /// How many topics are known (for the status bar / search hints).
  int get topicCount;

  /// The ROS type of [topic] (e.g. `sensor_msgs/msg/Image`), or null.
  String? typeOf(String topic);

  /// Start delivering decoded messages for [topic] to [cb].
  void subscribe(String topic, TopicListener cb);

  /// Stop delivering [topic] to [cb].
  void unsubscribe(String topic, TopicListener cb);

  /// Poll the graph for new/removed topics (no-op for push transports).
  void refreshGraph();

  /// Drive one iteration of work (no-op for push transports).
  void spinOnce();

  /// Tear down subscriptions/connections.
  void dispose();
}

/// Adapts rcldart's [ros.DynamicTopicHub] to [TopicSource].
class RclHub implements TopicSource {
  RclHub(this._d);
  final ros.DynamicTopicHub _d;

  @override
  Iterable<String> get topics => _d.topics;
  @override
  int get topicCount => _d.graph.length;
  @override
  String? typeOf(String topic) => _d.typeOf(topic);
  @override
  void subscribe(String topic, TopicListener cb) => _d.subscribe(topic, cb);
  @override
  void unsubscribe(String topic, TopicListener cb) => _d.unsubscribe(topic, cb);
  @override
  void refreshGraph() => _d.refreshGraph();
  @override
  void spinOnce() => _d.spinOnce();
  @override
  void dispose() {/* the underlying node owns the rcl handles */}
}
