/// dds_direct — a Node-like ROS 2 API that speaks DDS directly (CycloneDDS FFI)
/// and (de)serializes with pure-Dart [ros2_cdr]. No ROS install, no bridge.
///
/// ```dart
/// final node = Ros2Dds(domain: 0)
///   ..registerType('std_msgs/msg/String', 'string data');
/// final pub = node.advertise('chatter', 'std_msgs/msg/String');
/// pub.publish({'data': 'hello ROS 2'});
///
/// node.subscribe('chatter', 'std_msgs/msg/String', (msg) => print(msg['data']));
/// ```
library;

import 'dart:async';
import 'dart:ffi';
import 'dart:typed_data';

import 'package:ffi/ffi.dart';
import 'package:ros2_cdr/ros2_cdr.dart';

import 'src/ddsros_ffi.dart';

/// Maps a ROS type like `std_msgs/msg/String` to the DDS type name ROS 2 puts
/// on the wire: `std_msgs::msg::dds_::String_`.
String ddsTypeName(String rosType) {
  final p = rosType.split('/');
  final pkg = p.first;
  final name = p.last;
  final mid = p.length == 3 ? p[1] : 'msg';
  return '$pkg::$mid::dds_::${name}_';
}

class Ros2Dds {
  Ros2Dds({this.domain = 0}) : _n = DdsRosNative() {
    _pp = _n.participant(domain);
    if (_pp < 0) throw StateError('failed to create DDS participant ($_pp)');
  }

  final int domain;
  final DdsRosNative _n;
  late final int _pp;
  final MsgRegistry _registry = MsgRegistry();
  late final Ros2Codec _codec = Ros2Codec(_registry);
  final List<_Sub> _subs = [];
  int _disco = 0; // DCPSPublication reader (lazy)

  /// Register a message schema (concatenated ros2msg text, as from a .msg with
  /// its dependencies). Required before advertise/subscribe of that type.
  void registerType(String rosType, String schema) =>
      _registry.addConcatenated(rosType, schema);

  /// Bulk-register a whole schema registry (canonical `pkg/Type` -> concatenated
  /// ros2msg text), e.g. the bundled `assets/ros_schemas.json`. Lets subscribe
  /// decode ANY discovered type with no ROS on-device.
  void registerSchemas(Map<String, String> schemas) {
    schemas.forEach((type, text) => _registry.addConcatenated(type, text));
  }

  /// Discover the live ROS 2 graph over DDS (no bridge/WS): returns
  /// `{ '/topic': 'pkg/msg/Type' }` for every current publisher. Poll it.
  Map<String, String> discover() {
    if (_disco == 0) {
      _disco = _n.discoReader(_pp);
      if (_disco < 0) return const {};
    }
    const cap = 1 << 20;
    final buf = malloc<Uint8>(cap);
    try {
      final n = _n.discover(_disco, buf, cap);
      if (n <= 0) return const {};
      final out = <String, String>{};
      for (final line in String.fromCharCodes(buf.asTypedList(n)).split('\n')) {
        if (line.trim().isEmpty) continue;
        final p = line.split('\t');
        out[_ddsTopicToRos(p[0])] = p.length > 1 ? _ddsTypeToRos(p[1]) : '';
      }
      return out;
    } finally {
      malloc.free(buf);
    }
  }

  static String _ddsTopicToRos(String t) =>
      t.startsWith('rt/') ? '/${t.substring(3)}' : t;
  static String _ddsTypeToRos(String t) => t
      .replaceAll('::dds_::', '::')
      .replaceAll(RegExp(r'_$'), '')
      .replaceAll('::', '/');

  Ros2Publisher advertise(String topic, String rosType) {
    final w = _n.writer(_pp, topic.toNativeUtf8(), ddsTypeName(rosType).toNativeUtf8());
    if (w < 0) throw StateError('failed to create writer on $topic ($w)');
    return Ros2Publisher._(this, w, rosType);
  }

  StreamSubscription<void> subscribe(
      String topic, String rosType, void Function(Map<String, Object?>) onMsg,
      {Duration poll = const Duration(milliseconds: 10)}) {
    final r = _n.reader(_pp, topic.toNativeUtf8(), ddsTypeName(rosType).toNativeUtf8());
    if (r < 0) throw StateError('failed to create reader on $topic ($r)');
    const cap = 4 * 1024 * 1024;
    final buf = malloc<Uint8>(cap);
    final sub = _Sub(r, buf);
    _subs.add(sub);
    // Poll the reader; decode any sample with ros2_cdr.
    final timer = Timer.periodic(poll, (_) {
      while (true) {
        final n = _n.take(r, buf, cap);
        if (n <= 0) break;
        final cdr = Uint8List.fromList(buf.asTypedList(n));
        try {
          onMsg(_codec.decode(rosType, cdr));
        } catch (e) {
          onMsg({'_decode_error': '$e'});
        }
      }
    });
    return _TimerSubscription(timer, () {
      _n.del(r);
      malloc.free(buf);
      _subs.remove(sub);
    });
  }

  Uint8List encode(String rosType, Map<String, Object?> msg) =>
      _codec.encode(rosType, msg);

  void dispose() {
    for (final s in _subs) {
      malloc.free(s.buf);
    }
    _subs.clear();
    _n.del(_pp); // deletes children too
  }
}

class Ros2Publisher {
  Ros2Publisher._(this._node, this._w, this.rosType);
  final Ros2Dds _node;
  final int _w;
  final String rosType;

  void publish(Map<String, Object?> msg) {
    final cdr = _node.encode(rosType, msg);
    final p = malloc<Uint8>(cdr.length);
    p.asTypedList(cdr.length).setAll(0, cdr);
    try {
      _node._n.write(_w, p, cdr.length);
    } finally {
      malloc.free(p);
    }
  }
}

class _Sub {
  _Sub(this.reader, this.buf);
  final int reader;
  final Pointer<Uint8> buf;
}

class _TimerSubscription implements StreamSubscription<void> {
  _TimerSubscription(this._timer, this._onCancel);
  final Timer _timer;
  final void Function() _onCancel;
  @override
  Future<void> cancel() async {
    _timer.cancel();
    _onCancel();
  }

  @override
  void onData(void Function(void)? handleData) {}
  @override
  void onError(Function? handleError) {}
  @override
  void onDone(void Function()? handleDone) {}
  @override
  void pause([Future<void>? resumeSignal]) {}
  @override
  void resume() {}
  @override
  bool get isPaused => false;
  @override
  Future<E> asFuture<E>([E? futureValue]) => Completer<E>().future;
}
