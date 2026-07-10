// foxglove_ws.dart
//
// A pure-Dart Foxglove WebSocket transport for flutglove — the reliable way to
// see a computer's live ROS 2 topics from an Android device (emulator OR phone)
// WITHOUT any DDS-over-NAT pain and WITHOUT a bundled ROS closure.
//
// WHY THIS EXISTS (see docs / the connection dialog):
//   Native DDS discovery is UDP + multicast + bidirectional unicast. On the
//   Android emulator that runs behind QEMU user-mode NAT (SLIRP): the guest can
//   reach the host outbound, but the host CANNOT reach the guest's private
//   10.0.2.15 address, so the DDS discovery handshake never completes and the
//   computer's topics never appear — even with the peer IP set. (Proven
//   empirically: node comes up, own heartbeat works, host /tb1 topics never
//   discovered.) A single OUTBOUND TCP connection, however, works fine through
//   SLIRP. Foxglove's `foxglove_bridge` (ships with ROS 2, `ros2 run
//   foxglove_bridge foxglove_bridge`) speaks exactly that: one WebSocket over
//   TCP that surfaces every topic. So this client connects out to the bridge and
//   presents the same surface as [DynamicTopicHub] — the whole panel UI works
//   unchanged.
//
// It decodes messages generically from the ROS `.msg` schema the bridge sends
// (schemaEncoding "ros2msg"), so NO per-type typesupport / closure is needed on
// the device. That is the key difference from the rcl path (ros_cdr needs the
// introspection typesupport .so for each type).
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:web_socket_channel/web_socket_channel.dart';

import 'topic_source.dart';

/// Foxglove WebSocket client that implements the same [TopicSource] surface the
/// panels consume. Connect to `ws://<host>:8765`.
class FoxgloveWsHub implements TopicSource {
  FoxgloveWsHub(this.url);

  /// e.g. `ws://192.168.1.229:8765` (a real device) or `ws://10.0.2.2:8765`
  /// (the emulator reaching the host).
  final String url;

  WebSocketChannel? _ch;
  StreamSubscription? _sub;

  // channelId -> channel metadata
  final Map<int, _Channel> _channels = {};
  // topic name -> channelId
  final Map<String, int> _topicToChannel = {};
  // subscriptionId (we allocate) -> channelId, and reverse
  final Map<int, int> _subToChannel = {};
  final Map<int, int> _channelToSub = {};
  int _nextSubId = 1;

  // topic -> listeners
  final Map<String, List<void Function(Map<String, Object?>)>> _listeners = {};
  // parsed schema registry, keyed by canonical "pkg/Type"
  final Map<String, _MsgDef> _defs = {};
  // schemaName resolved to canonical root type, per channel
  final Map<int, String> _channelRootType = {};

  /// Connection state, surfaced in the status bar.
  String status = 'connecting…';
  bool connected = false;

  /// Called whenever the channel list changes, so the UI can redraw the sidebar.
  void Function()? onGraphChanged;

  Future<void> connect() async {
    try {
      // Offer both subprotocols: `foxglove.sdk.v1` (foxglove_bridge ≥ 3.x, the
      // Foxglove-SDK server) and the classic `foxglove.websocket.v1` (older
      // bridges). The server picks one; the advertise/subscribe/MessageData
      // framing is the same across both.
      _ch = WebSocketChannel.connect(
        Uri.parse(url),
        protocols: const ['foxglove.sdk.v1', 'foxglove.websocket.v1'],
      );
      await _ch!.ready;
      connected = true;
      status = 'connected';
      _sub = _ch!.stream.listen(_onFrame,
          onError: (e) => _fail('error: $e'), onDone: () => _fail('closed'));
      onGraphChanged?.call();
    } catch (e) {
      _fail('connect failed: $e');
      rethrow;
    }
  }

  void _fail(String why) {
    connected = false;
    status = why;
    onGraphChanged?.call();
  }

  // ---- incoming frames ------------------------------------------------------
  void _onFrame(dynamic data) {
    if (data is String) {
      _onJson(data);
    } else if (data is List<int>) {
      _onBinary(data is Uint8List ? data : Uint8List.fromList(data));
    }
  }

  void _onJson(String text) {
    Map<String, Object?> msg;
    try {
      msg = jsonDecode(text) as Map<String, Object?>;
    } catch (_) {
      return;
    }
    switch (msg['op']) {
      case 'serverInfo':
        status = 'connected: ${msg['name'] ?? 'foxglove_bridge'}';
        onGraphChanged?.call();
        break;
      case 'advertise':
        for (final c in (msg['channels'] as List? ?? const [])) {
          _addChannel(c as Map<String, Object?>);
        }
        onGraphChanged?.call();
        break;
      case 'unadvertise':
        for (final id in (msg['channelIds'] as List? ?? const [])) {
          _removeChannel((id as num).toInt());
        }
        onGraphChanged?.call();
        break;
      case 'status':
        status = 'bridge: ${msg['message']}';
        onGraphChanged?.call();
        break;
      default:
        // parameterValues / advertiseServices / etc. — ignored.
        break;
    }
  }

  void _addChannel(Map<String, Object?> c) {
    final id = (c['id'] as num).toInt();
    final ch = _Channel(
      id: id,
      topic: c['topic'] as String? ?? '',
      encoding: c['encoding'] as String? ?? 'cdr',
      schemaName: c['schemaName'] as String? ?? '',
      schema: c['schema'] as String? ?? '',
      schemaEncoding: c['schemaEncoding'] as String? ?? 'ros2msg',
    );
    _channels[id] = ch;
    _topicToChannel[ch.topic] = id;
    // Parse the schema lazily-but-once so decode is cheap per message.
    final root = _canonical(ch.schemaName, null);
    _channelRootType[id] = root;
    if (ch.encoding == 'cdr' &&
        (ch.schemaEncoding == 'ros2msg' || ch.schemaEncoding == 'ros2idl') &&
        !_defs.containsKey(root)) {
      try {
        _parseConcatenatedMsg(root, ch.schema);
      } catch (_) {
        // Unparseable schema — topic still lists, just won't decode.
      }
    }
    // If someone already asked to listen to this topic, subscribe now.
    if (_listeners.containsKey(ch.topic) && !_channelToSub.containsKey(id)) {
      _sendSubscribe(id);
    }
  }

  void _removeChannel(int id) {
    final ch = _channels.remove(id);
    if (ch != null) _topicToChannel.remove(ch.topic);
    final sub = _channelToSub.remove(id);
    if (sub != null) _subToChannel.remove(sub);
    _channelRootType.remove(id);
  }

  void _onBinary(Uint8List bytes) {
    if (bytes.isEmpty) return;
    final op = bytes[0];
    if (op != 0x01) return; // 0x01 = MessageData
    final bd = ByteData.sublistView(bytes);
    // [u8 op][u32 LE subId][u64 LE receiveTimestamp][payload...]
    final subId = bd.getUint32(1, Endian.little);
    final channelId = _subToChannel[subId];
    if (channelId == null) return;
    final ch = _channels[channelId];
    if (ch == null) return;
    final payload = Uint8List.sublistView(bytes, 13);
    final listeners = _listeners[ch.topic];
    if (listeners == null || listeners.isEmpty) return;
    Map<String, Object?> decoded;
    try {
      decoded = _decodeCdr(_channelRootType[channelId]!, payload);
    } catch (e) {
      decoded = {'_decode_error': '$e'};
    }
    for (final l in List.of(listeners)) {
      l(decoded);
    }
  }

  void _sendSubscribe(int channelId) {
    final subId = _nextSubId++;
    _subToChannel[subId] = channelId;
    _channelToSub[channelId] = subId;
    _ch?.sink.add(jsonEncode({
      'op': 'subscribe',
      'subscriptions': [
        {'id': subId, 'channelId': channelId}
      ],
    }));
  }

  void _sendUnsubscribe(int channelId) {
    final subId = _channelToSub.remove(channelId);
    if (subId == null) return;
    _subToChannel.remove(subId);
    _ch?.sink.add(jsonEncode({
      'op': 'unsubscribe',
      'subscriptionIds': [subId],
    }));
  }

  // ---- TopicSource surface --------------------------------------------------
  @override
  Iterable<String> get topics => _topicToChannel.keys;

  @override
  int get topicCount => _topicToChannel.length;

  @override
  String? typeOf(String topic) {
    final id = _topicToChannel[topic];
    return id == null ? null : _channels[id]!.schemaName;
  }

  @override
  void subscribe(String topic, void Function(Map<String, Object?>) cb) {
    (_listeners[topic] ??= []).add(cb);
    final id = _topicToChannel[topic];
    if (id != null && !_channelToSub.containsKey(id)) _sendSubscribe(id);
  }

  @override
  void unsubscribe(String topic, void Function(Map<String, Object?>) cb) {
    final ls = _listeners[topic];
    if (ls == null) return;
    ls.remove(cb);
    if (ls.isEmpty) {
      _listeners.remove(topic);
      final id = _topicToChannel[topic];
      if (id != null) _sendUnsubscribe(id);
    }
  }

  @override
  void refreshGraph() {/* push-based; channels arrive via advertise */}

  @override
  void spinOnce() {/* push-based; nothing to poll */}

  @override
  void dispose() {
    _sub?.cancel();
    _ch?.sink.close();
    _channels.clear();
    _topicToChannel.clear();
    _listeners.clear();
  }

  // ---- schema parsing (ros2msg concatenated) --------------------------------
  //
  // The bridge sends the root message definition followed by every dependency,
  // separated by lines of '=' with a "MSG: pkg/Type" header before each dep:
  //
  //   Header header
  //   float64 x
  //   ========================================
  //   MSG: std_msgs/Header
  //   builtin_interfaces/Time stamp
  //   string frame_id
  //   ...
  static final RegExp _sep = RegExp(r'^=+\s*$', multiLine: true);

  void _parseConcatenatedMsg(String rootType, String schema) {
    final blocks = schema.split(_sep);
    for (var i = 0; i < blocks.length; i++) {
      final block = blocks[i];
      final lines = block.split('\n');
      String? name;
      final bodyLines = <String>[];
      for (final raw in lines) {
        final line = raw.trim();
        if (line.startsWith('MSG:')) {
          name = _canonical(line.substring(4).trim(), null);
        } else {
          bodyLines.add(raw);
        }
      }
      // First block has no MSG header -> it's the root.
      final key = name ?? rootType;
      _defs[key] = _parseMsgBody(key, bodyLines);
    }
  }

  _MsgDef _parseMsgBody(String owner, List<String> lines) {
    final fields = <_Field>[];
    final pkg = owner.contains('/') ? owner.split('/').first : owner;
    for (var raw in lines) {
      // strip comments (but keep '#' inside string default values is rare —
      // ROS msg comments always start at a bare '#').
      final hash = raw.indexOf('#');
      if (hash >= 0) raw = raw.substring(0, hash);
      final line = raw.trim();
      if (line.isEmpty) continue;
      final parts = line.split(RegExp(r'\s+'));
      if (parts.length < 2) continue;
      var type = parts[0];
      final rest = parts[1];
      // Constant: "TYPE NAME = value" -> not serialized, skip.
      if (parts.length >= 3 && parts[2] == '=') continue;
      if (rest.contains('=')) continue;
      // Array suffix handling.
      bool isArray = false;
      int? fixedLen;
      final br = type.indexOf('[');
      if (br >= 0) {
        isArray = true;
        final inside = type.substring(br + 1, type.indexOf(']'));
        type = type.substring(0, br);
        if (inside.isNotEmpty && !inside.startsWith('<=')) {
          fixedLen = int.tryParse(inside);
        }
      }
      final bounded = type.indexOf('<=');
      if (bounded >= 0) type = type.substring(0, bounded); // string<=N
      fields.add(_Field(
        name: rest,
        type: _isPrimitive(type) ? type : _canonical(type, pkg),
        isPrimitive: _isPrimitive(type),
        isArray: isArray,
        fixedLen: fixedLen,
      ));
    }
    return _MsgDef(owner, fields);
  }

  static const _primitives = {
    'bool', 'byte', 'char', 'int8', 'uint8', 'int16', 'uint16', 'int32',
    'uint32', 'int64', 'uint64', 'float32', 'float64', 'string', 'wstring',
  };
  static bool _isPrimitive(String t) => _primitives.contains(t);

  /// Normalise a raw ROS type reference to canonical "pkg/Type".
  /// `parentPkg` resolves same-package short names.
  static String _canonical(String raw, String? parentPkg) {
    var t = raw.trim();
    final br = t.indexOf('[');
    if (br >= 0) t = t.substring(0, br);
    if (t == 'Header') return 'std_msgs/Header';
    final slashes = '/'.allMatches(t).length;
    if (slashes == 2) {
      // pkg/msg/Type -> pkg/Type
      final p = t.split('/');
      return '${p[0]}/${p[2]}';
    }
    if (slashes == 1) return t; // pkg/Type
    // bare Type -> same package
    return parentPkg == null ? t : '$parentPkg/$t';
  }

  // ---- CDR decode -----------------------------------------------------------
  Map<String, Object?> _decodeCdr(String rootType, Uint8List payload) {
    // Encapsulation header: [0x00, repId, options(2)]. repId bit0 = little.
    var little = true;
    var offset = 0;
    if (payload.length >= 4) {
      little = (payload[1] & 0x01) == 1;
      offset = 4; // skip encapsulation header; body alignment is relative here
    }
    final r = _Cdr(ByteData.sublistView(payload), offset, little);
    final def = _defs[rootType];
    if (def == null) return {'_no_schema': rootType};
    return _readStruct(def, r);
  }

  Map<String, Object?> _readStruct(_MsgDef def, _Cdr r) {
    final out = <String, Object?>{};
    for (final f in def.fields) {
      out[f.name] = f.isArray ? _readArray(f, r) : _readOne(f, r);
    }
    return out;
  }

  Object? _readArray(_Field f, _Cdr r) {
    final len = f.fixedLen ?? r.u32();
    // Fast path for uint8/byte/char/bool arrays -> typed bytes.
    if (f.isPrimitive &&
        (f.type == 'uint8' || f.type == 'byte' || f.type == 'char')) {
      return r.bytes(len);
    }
    final list = List<Object?>.filled(len, null, growable: false);
    for (var i = 0; i < len; i++) {
      list[i] = _readOne(f, r);
    }
    return list;
  }

  Object? _readOne(_Field f, _Cdr r) {
    if (!f.isPrimitive) {
      final def = _defs[f.type];
      if (def == null) return {'_no_schema': f.type};
      return _readStruct(def, r);
    }
    switch (f.type) {
      case 'bool':
        return r.u8() != 0;
      case 'byte':
      case 'uint8':
        return r.u8();
      case 'char':
        return r.u8();
      case 'int8':
        return r.i8();
      case 'int16':
        return r.i16();
      case 'uint16':
        return r.u16();
      case 'int32':
        return r.i32();
      case 'uint32':
        return r.u32();
      case 'int64':
        return r.i64();
      case 'uint64':
        return r.u64();
      case 'float32':
        return r.f32();
      case 'float64':
        return r.f64();
      case 'string':
      case 'wstring':
        return r.str();
      default:
        return null;
    }
  }
}

// ---- support types ----------------------------------------------------------
class _Channel {
  _Channel({
    required this.id,
    required this.topic,
    required this.encoding,
    required this.schemaName,
    required this.schema,
    required this.schemaEncoding,
  });
  final int id;
  final String topic;
  final String encoding;
  final String schemaName;
  final String schema;
  final String schemaEncoding;
}

class _MsgDef {
  _MsgDef(this.name, this.fields);
  final String name;
  final List<_Field> fields;
}

class _Field {
  _Field({
    required this.name,
    required this.type,
    required this.isPrimitive,
    required this.isArray,
    this.fixedLen,
  });
  final String name;
  final String type; // primitive keyword OR canonical "pkg/Type"
  final bool isPrimitive;
  final bool isArray;
  final int? fixedLen; // null => sequence (length-prefixed)
}

/// Minimal aligned CDR (XCDR1) reader. Alignment is relative to [_base] — the
/// start of the encapsulation body (after the 4-byte header).
class _Cdr {
  _Cdr(this._d, this._base, this._little) : _p = _base;
  final ByteData _d;
  final int _base;
  final bool _little;
  int _p;

  Endian get _e => _little ? Endian.little : Endian.big;

  void _align(int n) {
    final rel = _p - _base;
    final pad = (n - (rel % n)) % n;
    _p += pad;
  }

  int u8() => _d.getUint8(_p++);
  int i8() => _d.getInt8(_p++);
  int u16() {
    _align(2);
    final v = _d.getUint16(_p, _e);
    _p += 2;
    return v;
  }

  int i16() {
    _align(2);
    final v = _d.getInt16(_p, _e);
    _p += 2;
    return v;
  }

  int u32() {
    _align(4);
    final v = _d.getUint32(_p, _e);
    _p += 4;
    return v;
  }

  int i32() {
    _align(4);
    final v = _d.getInt32(_p, _e);
    _p += 4;
    return v;
  }

  int u64() {
    _align(8);
    final v = _d.getUint64(_p, _e);
    _p += 8;
    return v;
  }

  int i64() {
    _align(8);
    final v = _d.getInt64(_p, _e);
    _p += 8;
    return v;
  }

  double f32() {
    _align(4);
    final v = _d.getFloat32(_p, _e);
    _p += 4;
    return v;
  }

  double f64() {
    _align(8);
    final v = _d.getFloat64(_p, _e);
    _p += 8;
    return v;
  }

  String str() {
    final len = u32(); // includes null terminator
    if (len == 0) return '';
    final b = Uint8List.sublistView(_d, _p, _p + len - 1);
    _p += len; // consume terminator too
    return utf8.decode(b, allowMalformed: true);
  }

  Uint8List bytes(int n) {
    final b = Uint8List.fromList(
        Uint8List.sublistView(_d, _p, _p + n)); // copy out
    _p += n;
    return b;
  }
}
