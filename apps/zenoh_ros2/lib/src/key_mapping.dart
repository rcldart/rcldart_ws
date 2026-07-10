// key_mapping.dart — ROS 2 topic <-> Zenoh key expression.
//
// The exact key depends on what is on the robot side:
//  * zenoh-bridge-ros2dds (default): a topic "/chatter" is exposed at key
//    "chatter" (leading slash stripped), optionally under a scope/namespace.
//  * rmw_zenoh: a mangled keyexpr "<domain>/<mangled_topic>/<type>/<type_hash>".
//
// Default here targets zenoh-bridge-ros2dds. Pass a custom [KeyMapper] for
// rmw_zenoh or a scoped bridge.
typedef KeyMapper = String Function(String topic, String rosType);

/// zenoh-bridge-ros2dds default: strip the leading '/', keep the rest.
/// An optional [scope] is prepended (matching the bridge's `--scope`).
class BridgeKeyMapper {
  const BridgeKeyMapper({this.scope});
  final String? scope;

  String call(String topic, String rosType) {
    final t = topic.startsWith('/') ? topic.substring(1) : topic;
    return scope == null || scope!.isEmpty ? t : '$scope/$t';
  }
}

/// A subscribe pattern that also matches the type-suffixed keys some bridge
/// configs use (`chatter/**`).
String subscribePattern(String key) => key.contains('*') ? key : key;
