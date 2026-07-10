// flutglove_foxglove — a minimal ROS 2 subscribe+publish frontend over the
// Foxglove WebSocket bridge. Same tabbed shape as flutglove_dds; discovery +
// per-message schemas come from the bridge, so subscribe/decode needs nothing
// bundled. Publish serializes JSON with ros2_cdr and uses the bridge's
// clientPublish capability.
import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:ros2_cdr/ros2_cdr.dart';

import 'foxglove_ws.dart';
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
  static const accent = Color(0xFFF7813C); // foxglove = orange
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
      title: 'flutglove_foxglove',
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
  FoxgloveWsHub? _hub;
  Map<String, String> _schemas = {};
  final _codec = Ros2Codec(MsgRegistry());

  final _url = TextEditingController(text: 'ws://127.0.0.1:8765');
  bool _connected = false;
  String _connStatus = 'not connected';

  final _filter = TextEditingController();
  String? _selected;
  Map<String, Object?>? _last;

  final _pubTopic = TextEditingController(text: '/flutglove_foxglove/chatter');
  final _pubType = TextEditingController(text: 'std_msgs/msg/String');
  final _pubBody = TextEditingController(text: '{\n  "data": "hello from flutglove_foxglove"\n}');
  final Map<String, int> _pubChannels = {};
  String _pubStatus = '';
  int _pubCount = 0;

  @override
  void initState() {
    super.initState();
    _loadSchemas();
  }

  Future<void> _loadSchemas() async {
    _schemas = await loadBundledSchemas();
    _schemas.forEach((k, v) => _codec.registry.addConcatenated(k, v));
  }

  Future<void> _connect() async {
    setState(() => _connStatus = 'connecting…');
    final hub = FoxgloveWsHub(_url.text.trim())
      ..onGraphChanged = () {
        if (mounted) setState(() {});
      };
    try {
      await hub.connect();
      setState(() {
        _hub = hub;
        _connected = hub.connected;
        _connStatus = hub.status;
      });
    } catch (e) {
      setState(() => _connStatus = 'failed: $e');
    }
  }

  void _select(String topic) {
    if (_selected != null) _hub?.unsubscribe(_selected!, _onMsg);
    setState(() {
      _selected = topic;
      _last = null;
    });
    _hub?.subscribe(topic, _onMsg);
  }

  void _onMsg(Map<String, Object?> m) {
    if (mounted) setState(() => _last = m);
  }

  Future<void> _publish() async {
    final hub = _hub;
    if (hub == null) return;
    final topic = _pubTopic.text.trim();
    final type = _pubType.text.trim();
    try {
      final body = (jsonDecode(_pubBody.text) as Map).cast<String, Object?>();
      final cdr = _codec.encode(type, body);
      final ch = _pubChannels[topic] ??=
          hub.advertiseClient(topic, type, _schemas[canonicalType(type)] ?? 'string data');
      hub.publishRaw(ch, cdr);
      setState(() => _pubStatus = '✓ published #${++_pubCount} to $topic');
    } catch (e) {
      setState(() => _pubStatus = '✗ $e');
    }
  }

  @override
  void dispose() {
    _tabs.dispose();
    _hub?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final q = _filter.text.trim().toLowerCase();
    final topics = (_hub?.topics.toList() ?? [])
      ..sort();
    final filtered = topics
        .where((t) => q.isEmpty || t.toLowerCase().contains(q) ||
            (_hub?.typeOf(t)?.toLowerCase().contains(q) ?? false))
        .toList();
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
                ? const Center(child: Text('Connect to a Foxglove bridge first', style: TextStyle(color: Fx.dim)))
                : TabBarView(controller: _tabs, children: [_subscribeTab(filtered), _publishTab()]),
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
          Icon(Icons.hub, color: Fx.accent, size: 20),
          SizedBox(width: 8),
          Text('flutglove_foxglove', style: TextStyle(fontWeight: FontWeight.w600, fontSize: 15)),
          SizedBox(width: 8),
          Text('Foxglove WS · ros2_cdr', style: TextStyle(fontSize: 10.5, color: Fx.dim)),
        ]),
      );

  Widget _connBar() => Container(
        color: Fx.surface,
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        child: Row(children: [
          Expanded(child: _field(_url, hint: 'ws://<host>:8765')),
          const SizedBox(width: 10),
          FilledButton(onPressed: _connect, child: Text(_connected ? 'Reconnect' : 'Connect')),
          const SizedBox(width: 10),
          Icon(Icons.circle, size: 8, color: _connected ? Fx.ok : Fx.dim),
          const SizedBox(width: 5),
          Text(_connected ? '${_hub?.topicCount ?? 0} topics' : _connStatus,
              style: const TextStyle(fontSize: 11, color: Fx.dim)),
        ]),
      );

  Widget _subscribeTab(List<String> topics) => Row(children: [
        SizedBox(
          width: 320,
          child: Column(children: [
            Padding(
              padding: const EdgeInsets.all(8),
              child: _field(_filter, hint: 'Filter topics…', onChanged: () => setState(() {})),
            ),
            Expanded(
              child: ListView.builder(
                itemCount: topics.length,
                itemBuilder: (_, i) {
                  final t = topics[i];
                  final sel = t == _selected;
                  return InkWell(
                    onTap: () => _select(t),
                    child: Container(
                      color: sel ? Fx.surface2 : null,
                      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                        Text(t, style: TextStyle(fontSize: 13, color: sel ? Fx.accent : Fx.text)),
                        Text(_hub?.typeOf(t) ?? '', style: const TextStyle(fontSize: 10.5, color: Fx.dim)),
                      ]),
                    ),
                  );
                },
              ),
            ),
          ]),
        ),
        Container(width: 1, color: Fx.border),
        Expanded(
          child: _selected == null
              ? const Center(child: Text('Select a topic', style: TextStyle(color: Fx.dim)))
              : Padding(
                  padding: const EdgeInsets.all(12),
                  child: _last == null
                      ? const Center(child: Text('waiting for data…', style: TextStyle(color: Fx.dim)))
                      : SingleChildScrollView(child: _tree(_last!, 0)),
                ),
        ),
      ]);

  Widget _publishTab() => Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 640),
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(20),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              const Text('Publish over the Foxglove bridge', style: TextStyle(fontSize: 15, fontWeight: FontWeight.w600)),
              const SizedBox(height: 4),
              const Text('ros2_cdr serializes the JSON; the bridge\'s clientPublish '
                  'capability forwards it to a ROS 2 publisher.',
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

  Widget _field(TextEditingController c, {String? hint, int lines = 1, bool mono = false, VoidCallback? onChanged}) => TextField(
        controller: c,
        maxLines: lines,
        onChanged: onChanged == null ? null : (_) => onChanged(),
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
