/// zenoh_ros2 — a Node-like ROS 2 API over Zenoh + pure-Dart [ros2_cdr].
///
/// ```dart
/// final node = await Ros2Zenoh.open(connect: ['tcp/192.168.1.50:7447'])
///   ..registerType('std_msgs/msg/String', 'string data');
/// await node.subscribe('/chatter', 'std_msgs/msg/String',
///     (msg) => print(msg['data']));
/// final pub = await node.advertise('/chatter', 'std_msgs/msg/String');
/// await pub.publish({'data': 'hello'});
/// ```
library;

import 'dart:async';
import 'dart:typed_data';

import 'package:ros2_cdr/ros2_cdr.dart';
import 'package:zenoh_ffi/zenoh_ffi.dart';

import 'src/key_mapping.dart';

export 'src/key_mapping.dart';

class Ros2Zenoh {
  Ros2Zenoh._(this._session, this._mapper);

  final ZenohSession _session;
  final Object _mapper; // BridgeKeyMapper or a KeyMapper function
  final MsgRegistry _registry = MsgRegistry();
  late final Ros2Codec _codec = Ros2Codec(_registry);

  /// Opens a Zenoh session. [connect] are router/peer endpoints
  /// (e.g. `['tcp/192.168.1.50:7447']`) — one outbound TCP, NAT-friendly.
  static Future<Ros2Zenoh> open({
    List<String> connect = const [],
    Object mapper = const BridgeKeyMapper(),
  }) async {
    final session = connect.isEmpty
        ? await ZenohSession.open()
        : await ZenohSession.openWithConfig(
            ZenohConfigBuilder().connect(connect));
    return Ros2Zenoh._(session, mapper);
  }

  String _key(String topic, String rosType) {
    final m = _mapper;
    if (m is BridgeKeyMapper) return m.call(topic, rosType);
    return (m as KeyMapper)(topic, rosType);
  }

  /// Register a message schema (concatenated ros2msg text). Required before
  /// advertise/subscribe of that type.
  void registerType(String rosType, String schema) =>
      _registry.addConcatenated(rosType, schema);

  /// Subscribe: each Zenoh sample's payload is the raw CDR, decoded with
  /// ros2_cdr into a Dart map.
  Future<StreamSubscription<ZenohSample>> subscribe(
      String topic, String rosType, void Function(Map<String, Object?>) onMsg) async {
    final sub = await _session.declareSubscriber(_key(topic, rosType));
    return sub.stream.listen((sample) {
      try {
        onMsg(_codec.decode(rosType, sample.payload));
      } catch (e) {
        onMsg({'_decode_error': '$e'});
      }
    });
  }

  Future<Ros2ZenohPublisher> advertise(String topic, String rosType) async {
    final pub = await _session.declarePublisher(_key(topic, rosType));
    return Ros2ZenohPublisher._(this, pub, rosType);
  }

  Uint8List encode(String rosType, Map<String, Object?> msg) =>
      _codec.encode(rosType, msg);

  Future<void> close() => _session.close();
}

class Ros2ZenohPublisher {
  Ros2ZenohPublisher._(this._node, this._pub, this.rosType);
  final Ros2Zenoh _node;
  final ZenohPublisher _pub;
  final String rosType;

  Future<void> publish(Map<String, Object?> msg) =>
      _pub.put(_node.encode(rosType, msg));
}
