// topic_source.dart — the surface the UI consumes. Minimal (WS-only) variant:
// no rcldart dependency, just the interface FoxgloveWsHub implements.
typedef TopicListener = void Function(Map<String, Object?> message);

abstract class TopicSource {
  Iterable<String> get topics;
  int get topicCount;
  String? typeOf(String topic);
  void subscribe(String topic, TopicListener cb);
  void unsubscribe(String topic, TopicListener cb);
  void refreshGraph();
  void spinOnce();
  void dispose();
}
