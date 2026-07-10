// flutglove_zenoh — a minimal ROS 2 subscribe+publish frontend over ZENOH.
// Same tabbed shape as flutglove_dds, but the transport layer is zenoh_ros2
// (Zenoh via a zenoh-bridge-ros2dds). Serialization is pure-Dart ros2_cdr.
//
// Zenoh has no DDS-style graph discovery here, so topics are entered by hand —
// the point is the SAME architecture over a different transport layer.
import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:zenoh_ros2/zenoh_ros2.dart';

import 'schemas.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const App());
}

class Fx {
  static const bg = Color(0xFF0E1015);
  static const surface = Color(0xFF161922);
  static const surface2 = Color(0xFF1C2029);
  static const rail = Color(0xFF101219);
  static const border = Color(0xFF2A303C);
  static const accent = Color(0xFF7C5CF7); // zenoh = purple
  static const text = Color(0xFFE6E9EF);
  static const dim = Color(0xFF8A93A3);
  static const ok = Color(0xFF3DD68C);
}

class App extends StatelessWidget {
  const App({super.key});
  @override
  Widget build(BuildContext context) {
    final base = ThemeData.dark(useMaterial3: true);
    return MaterialApp(
      title: 'flutglove_zenoh',
      debugShowCheckedModeBanner: false,
      theme: base.copyWith(
        scaffoldBackgroundColor: Fx.bg,
        colorScheme: base.colorScheme.copyWith(primary: Fx.accent, surface: Fx.surface),
      ),
      home: const _Home(),
    );
  }
}

class _Home extends StatefulWidget {
  const _Home();
  @override
  State<_Home> createState() => _HomeState();
}

class _HomeState extends State<_Home> with SingleTickerProviderStateMixin {
  late final TabController _tabs = TabController(length: 2, vsync: this);
  Ros2Zenoh? _node;
  Map<String, String> _schemas = {};
  String _connStatus = 'not connected';
  bool _connected = false;

  final _connect = TextEditingController(text: 'tcp/127.0.0.1:7447');

  // subscribe state
  final _subTopic = TextEditingController(text: '/chatter');
  final _subType = TextEditingController(text: 'std_msgs/msg/String');
  Map<String, Object?>? _last;
  String _subInfo = '';
  StreamSubscription<dynamic>? _sub;

  // publish state
  final _pubTopic = TextEditingController(text: '/flutglove_zenoh/chatter');
  final _pubType = TextEditingController(text: 'std_msgs/msg/String');
  final _pubBody = TextEditingController(text: '{\n  "data": "hello from flutglove_zenoh"\n}');
  final Map<String, Ros2ZenohPublisher> _pubs = {};
  String _pubStatus = '';
  int _pubCount = 0;

  @override
  void initState() {
    super.initState();
    _loadSchemas();
  }

  Future<void> _loadSchemas() async {
    _schemas = await loadBundledSchemas();
  }

  Future<void> _connectZenoh() async {
    setState(() => _connStatus = 'connecting…');
    try {
      final endpoints = _connect.text.trim().isEmpty ? <String>[] : [_connect.text.trim()];
      final node = await Ros2Zenoh.open(connect: endpoints);
      _schemas.forEach(node.registerType);
      setState(() {
        _node = node;
        _connected = true;
        _connStatus = endpoints.isEmpty ? 'connected (local)' : 'connected ${endpoints.first}';
      });
    } catch (e) {
      setState(() => _connStatus = 'failed: $e');
    }
  }

  Future<void> _subscribe() async {
    if (_node == null) return;
    await _sub?.cancel();
    final topic = _subTopic.text.trim();
    final type = _subType.text.trim();
    setState(() {
      _last = null;
      _subInfo = 'subscribing to $topic …';
    });
    try {
      _sub = await _node!.subscribe(topic, type, (m) {
        if (mounted) setState(() => _last = m);
      });
      setState(() => _subInfo = 'subscribed to $topic  [$type]');
    } catch (e) {
      setState(() => _subInfo = 'error: $e');
    }
  }

  Future<void> _publish() async {
    if (_node == null) return;
    final topic = _pubTopic.text.trim();
    final type = _pubType.text.trim();
    try {
      final body = (jsonDecode(_pubBody.text) as Map).cast<String, Object?>();
      final key = '$topic|$type';
      final pub = _pubs[key] ??= await _node!.advertise(topic, type);
      await pub.publish(body);
      setState(() => _pubStatus = '✓ published #${++_pubCount} to $topic');
    } catch (e) {
      setState(() => _pubStatus = '✗ $e');
    }
  }

  @override
  void dispose() {
    _tabs.dispose();
    _sub?.cancel();
    _node?.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Column(children: [
          _topBar(),
          _connBar(),
          Container(
            color: Fx.rail,
            child: TabBar(
              controller: _tabs,
              indicatorColor: Fx.accent,
              labelColor: Fx.accent,
              unselectedLabelColor: Fx.dim,
              tabs: const [Tab(text: 'Subscribe'), Tab(text: 'Publish')],
            ),
          ),
          Container(height: 1, color: Fx.border),
          Expanded(
            child: !_connected
                ? const Center(child: Text('Connect to a Zenoh router first', style: TextStyle(color: Fx.dim)))
                : TabBarView(controller: _tabs, children: [_subscribeTab(), _publishTab()]),
          ),
        ]),
      ),
    );
  }

  Widget _topBar() => Container(
        height: 44,
        color: Fx.rail,
        padding: const EdgeInsets.symmetric(horizontal: 12),
        child: Row(children: const [
          Icon(Icons.router, color: Fx.accent, size: 20),
          SizedBox(width: 8),
          Text('flutglove_zenoh', style: TextStyle(fontWeight: FontWeight.w600, fontSize: 15)),
          SizedBox(width: 8),
          Text('Zenoh · zenoh_ros2 + ros2_cdr', style: TextStyle(fontSize: 10.5, color: Fx.dim)),
        ]),
      );

  Widget _connBar() => Container(
        color: Fx.surface,
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        child: Row(children: [
          Expanded(child: _field(_connect, hint: 'tcp/<robot-ip>:7447')),
          const SizedBox(width: 10),
          FilledButton(onPressed: _connectZenoh, child: Text(_connected ? 'Reconnect' : 'Connect')),
          const SizedBox(width: 10),
          Icon(Icons.circle, size: 8, color: _connected ? Fx.ok : Fx.dim),
          const SizedBox(width: 5),
          Text(_connStatus, style: const TextStyle(fontSize: 11, color: Fx.dim)),
        ]),
      );

  Widget _subscribeTab() => Padding(
        padding: const EdgeInsets.all(16),
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Row(children: [
            Expanded(flex: 3, child: _field(_subTopic, hint: '/chatter')),
            const SizedBox(width: 8),
            Expanded(flex: 3, child: _field(_subType, hint: 'std_msgs/msg/String')),
            const SizedBox(width: 8),
            FilledButton.icon(onPressed: _subscribe, icon: const Icon(Icons.download, size: 16), label: const Text('Subscribe')),
          ]),
          const SizedBox(height: 6),
          Text(_subInfo, style: const TextStyle(fontSize: 11, color: Fx.dim)),
          const Divider(color: Fx.border),
          Expanded(
            child: _last == null
                ? const Center(child: Text('waiting for data…', style: TextStyle(color: Fx.dim)))
                : SingleChildScrollView(child: _tree(_last!, 0)),
          ),
        ]),
      );

  Widget _publishTab() => Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 640),
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(20),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              const Text('Publish over Zenoh', style: TextStyle(fontSize: 15, fontWeight: FontWeight.w600)),
              const SizedBox(height: 4),
              const Text('ros2_cdr serializes the JSON to CDR; zenoh_ros2 puts it on the '
                  'mapped key. The bridge forwards it to the ROS 2 graph.',
                  style: TextStyle(fontSize: 11, color: Fx.dim)),
              const SizedBox(height: 16),
              _lbl('TOPIC'), _field(_pubTopic),
              const SizedBox(height: 12),
              _lbl('TYPE'), _field(_pubType),
              const SizedBox(height: 12),
              _lbl('MESSAGE (JSON)'), _field(_pubBody, lines: 8, mono: true),
              const SizedBox(height: 16),
              Row(children: [
                FilledButton.icon(onPressed: _publish, icon: const Icon(Icons.send, size: 16), label: const Text('Publish')),
                const SizedBox(width: 12),
                Expanded(child: Text(_pubStatus, style: TextStyle(fontSize: 12, color: _pubStatus.startsWith('✓') ? Fx.ok : Fx.dim))),
              ]),
            ]),
          ),
        ),
      );

  Widget _lbl(String s) => Padding(padding: const EdgeInsets.only(bottom: 4), child: Text(s, style: const TextStyle(fontSize: 11, color: Fx.dim)));

  Widget _field(TextEditingController c, {String? hint, int lines = 1, bool mono = false}) => TextField(
        controller: c,
        maxLines: lines,
        style: TextStyle(fontSize: 13, fontFamily: mono ? 'monospace' : null),
        decoration: InputDecoration(
          hintText: hint, isDense: true, filled: true, fillColor: Fx.surface2,
          border: OutlineInputBorder(borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none),
        ),
      );

  Widget _tree(Object? v, int depth) {
    final pad = EdgeInsets.only(left: depth * 14.0);
    if (v is Map) {
      return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        for (final e in v.entries)
          Padding(
            padding: pad,
            child: (e.value is Map || e.value is List)
                ? Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                    Text('${e.key}:', style: const TextStyle(fontFamily: 'monospace', fontSize: 12.5, color: Fx.dim)),
                    _tree(e.value, depth + 1),
                  ])
                : Text('${e.key}: ${e.value}', style: const TextStyle(fontFamily: 'monospace', fontSize: 12.5, color: Fx.text)),
          ),
      ]);
    }
    if (v is List) {
      final n = v.length;
      return Padding(padding: pad, child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Text('[$n]', style: const TextStyle(fontFamily: 'monospace', fontSize: 12, color: Fx.dim)),
        for (var i = 0; i < n && i < 8; i++) _tree(v[i], depth + 1),
      ]));
    }
    return Padding(padding: pad, child: Text('$v', style: const TextStyle(fontFamily: 'monospace', fontSize: 12.5, color: Fx.text)));
  }
}
