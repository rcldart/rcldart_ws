// flutglove_dds — a minimal Foxglove-style ROS 2 viewer that pulls the live
// graph DIRECTLY over CycloneDDS (dds_direct + ros2_cdr). No bridge, no
// WebSocket, no ROS install: it discovers topics over DCPSPublication and
// decodes every message with the bundled .msg schema registry.
//
// This is a deliberately SIMPLE second frontend over the same DDS core that
// flutglove uses — the point is to grow different project structures on the
// cyclone_dds layer.
import 'dart:async';
import 'dart:convert';

import 'package:dds_direct/dds_direct.dart';
import 'package:dds_direct/schemas.dart';
import 'package:flutter/material.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const FlutgloveDdsApp());
}

// --- palette ----------------------------------------------------------------
class Fx {
  static const bg = Color(0xFF0E1015);
  static const surface = Color(0xFF161922);
  static const surface2 = Color(0xFF1C2029);
  static const rail = Color(0xFF101219);
  static const border = Color(0xFF2A303C);
  static const accent = Color(0xFF4C82F7);
  static const text = Color(0xFFE6E9EF);
  static const dim = Color(0xFF8A93A3);
  static const ok = Color(0xFF3DD68C);
}

class FlutgloveDdsApp extends StatelessWidget {
  const FlutgloveDdsApp({super.key});
  @override
  Widget build(BuildContext context) {
    final base = ThemeData.dark(useMaterial3: true);
    return MaterialApp(
      title: 'flutglove_dds',
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
  Ros2Dds? _node;
  String? _error;
  final int _domain = 0;

  Map<String, String> _graph = {}; // '/topic' -> 'pkg/msg/Type'
  final _filter = TextEditingController();
  String? _selected;
  Map<String, Object?>? _last;
  int _msgCount = 0, _rate = 0, _lastCount = 0;
  StreamSubscription<void>? _sub;
  Timer? _discoTimer, _rateTimer;

  late final TabController _tabs = TabController(length: 2, vsync: this);

  // --- publish (serialization) state ---
  final _pubTopic = TextEditingController(text: '/flutglove_dds/chatter');
  final _pubType = TextEditingController(text: 'std_msgs/msg/String');
  final _pubBody = TextEditingController(text: '{\n  "data": "hello from flutglove_dds"\n}');
  final Map<String, Ros2Publisher> _pubs = {};
  String _pubStatus = '';

  @override
  void initState() {
    super.initState();
    _start();
  }

  Future<void> _start() async {
    try {
      final node = Ros2Dds(domain: _domain);
      node.registerSchemas(await loadBundledSchemas());
      _node = node;
      _refresh();
      _discoTimer = Timer.periodic(const Duration(milliseconds: 1500), (_) => _refresh());
      _rateTimer = Timer.periodic(const Duration(seconds: 1), (_) {
        if (!mounted) return;
        setState(() {
          _rate = _msgCount - _lastCount;
          _lastCount = _msgCount;
        });
      });
    } catch (e) {
      setState(() => _error = '$e');
    }
  }

  void _refresh() {
    final g = _node?.discover() ?? const {};
    if (!mounted) return;
    setState(() => _graph = Map.fromEntries(
        g.entries.toList()..sort((a, b) => a.key.compareTo(b.key))));
  }

  void _select(String topic) {
    _sub?.cancel();
    setState(() {
      _selected = topic;
      _last = null;
    });
    final type = _graph[topic];
    if (type == null) return;
    final ddsTopic = topic.startsWith('/') ? topic.substring(1) : topic;
    _sub = _node!.subscribe(ddsTopic, type, (m) {
      _msgCount++;
      if (mounted) setState(() => _last = m);
    });
  }

  // Serialize a Dart map to CDR and publish it over DDS (ros2_cdr encode).
  int _pubCount = 0;
  void _publish() {
    final topic = _pubTopic.text.trim();
    final type = _pubType.text.trim();
    if (_node == null) return;
    try {
      final body = (jsonDecode(_pubBody.text) as Map).cast<String, Object?>();
      final key = '$topic|$type';
      final pub = _pubs[key] ??= _node!.advertise(
          topic.startsWith('/') ? topic.substring(1) : topic, type);
      pub.publish(body);
      setState(() => _pubStatus = '✓ published #${++_pubCount} to $topic');
    } catch (e) {
      setState(() => _pubStatus = '✗ $e');
    }
  }

  @override
  void dispose() {
    _discoTimer?.cancel();
    _rateTimer?.cancel();
    _sub?.cancel();
    _tabs.dispose();
    _node?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (_error != null) {
      return Scaffold(
        body: Center(
          child: Padding(
            padding: const EdgeInsets.all(24),
            child: Text('CycloneDDS init failed:\n$_error',
                textAlign: TextAlign.center, style: const TextStyle(color: Fx.dim)),
          ),
        ),
      );
    }
    final q = _filter.text.trim().toLowerCase();
    final topics = _graph.keys
        .where((t) => q.isEmpty || t.toLowerCase().contains(q) ||
            (_graph[t]?.toLowerCase().contains(q) ?? false))
        .toList();
    return Scaffold(
      body: SafeArea(
        child: Column(children: [
          _topBar(),
          Container(
            color: Fx.rail,
            child: TabBar(
              controller: _tabs,
              isScrollable: true,
              indicatorColor: Fx.accent,
              labelColor: Fx.accent,
              unselectedLabelColor: Fx.dim,
              tabs: const [
                Tab(height: 36, child: Row(mainAxisSize: MainAxisSize.min, children: [
                  Icon(Icons.download, size: 15), SizedBox(width: 6), Text('Subscribe'),
                ])),
                Tab(height: 36, child: Row(mainAxisSize: MainAxisSize.min, children: [
                  Icon(Icons.upload, size: 15), SizedBox(width: 6), Text('Publish'),
                ])),
              ],
            ),
          ),
          Container(height: 1, color: Fx.border),
          Expanded(
            child: TabBarView(
              controller: _tabs,
              children: [
                Row(children: [
                  SizedBox(width: 320, child: _sidebar(topics)),
                  Container(width: 1, color: Fx.border),
                  Expanded(child: _viewer()),
                ]),
                _publishTab(),
              ],
            ),
          ),
          Container(height: 1, color: Fx.border),
          _statusBar(),
        ]),
      ),
    );
  }

  Widget _topBar() => Container(
        height: 46,
        color: Fx.rail,
        padding: const EdgeInsets.symmetric(horizontal: 12),
        child: Row(children: const [
          Icon(Icons.lan, color: Fx.accent, size: 20),
          SizedBox(width: 8),
          Text('flutglove_dds', style: TextStyle(fontWeight: FontWeight.w600, fontSize: 15)),
          SizedBox(width: 8),
          Text('CycloneDDS · no bridge · no WS',
              style: TextStyle(fontSize: 10.5, color: Fx.dim)),
        ]),
      );

  Widget _sidebar(List<String> topics) => Column(children: [
        Padding(
          padding: const EdgeInsets.all(8),
          child: TextField(
            controller: _filter,
            onChanged: (_) => setState(() {}),
            style: const TextStyle(fontSize: 13),
            decoration: InputDecoration(
              isDense: true,
              hintText: 'Filter ${_graph.length} topics…',
              prefixIcon: const Icon(Icons.search, size: 18, color: Fx.dim),
              filled: true,
              fillColor: Fx.surface2,
              border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none),
            ),
          ),
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
                    Text(_graph[t] ?? '', style: const TextStyle(fontSize: 10.5, color: Fx.dim)),
                  ]),
                ),
              );
            },
          ),
        ),
      ]);

  Widget _viewer() {
    if (_selected == null) {
      return const Center(
          child: Text('Select a topic to view live messages',
              style: TextStyle(color: Fx.dim)));
    }
    return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      Container(
        width: double.infinity,
        color: Fx.surface,
        padding: const EdgeInsets.all(10),
        child: Text('$_selected   [${_graph[_selected]}]',
            style: const TextStyle(fontSize: 13, color: Fx.accent)),
      ),
      Expanded(
        child: _last == null
            ? const Center(child: Text('waiting for data…', style: TextStyle(color: Fx.dim)))
            : SingleChildScrollView(
                padding: const EdgeInsets.all(12),
                child: _tree(_last!, 0),
              ),
      ),
    ]);
  }

  // Publish tab: serialize a Dart/JSON map to CDR and send it over DDS.
  Widget _publishTab() => Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 640),
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(20),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              const Text('Publish a message over DDS',
                  style: TextStyle(fontSize: 15, fontWeight: FontWeight.w600)),
              const SizedBox(height: 4),
              const Text('ros2_cdr serializes the JSON body to CDR; dds_direct writes '
                  'it on the ROS topic. Any ROS 2 subscriber on the domain receives it.',
                  style: TextStyle(fontSize: 11, color: Fx.dim)),
              const SizedBox(height: 16),
              _label('TOPIC'),
              _field(_pubTopic, hint: '/flutglove_dds/chatter'),
              const SizedBox(height: 12),
              _label('TYPE'),
              _field(_pubType, hint: 'std_msgs/msg/String'),
              const SizedBox(height: 12),
              _label('MESSAGE (JSON matching the type\'s fields)'),
              _field(_pubBody, hint: '{ "data": "hi" }', lines: 8, mono: true),
              const SizedBox(height: 16),
              Row(children: [
                FilledButton.icon(
                  onPressed: _publish,
                  icon: const Icon(Icons.send, size: 16),
                  label: const Text('Publish'),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Text(_pubStatus,
                      style: TextStyle(
                          fontSize: 12,
                          color: _pubStatus.startsWith('✓') ? Fx.ok : Fx.dim)),
                ),
              ]),
              const SizedBox(height: 12),
              const Text('Examples:', style: TextStyle(fontSize: 11, color: Fx.dim)),
              _example('std_msgs/msg/String', '{ "data": "hello" }'),
              _example('geometry_msgs/msg/Twist',
                  '{ "linear": {"x":0.2,"y":0,"z":0}, "angular": {"x":0,"y":0,"z":0.5} }'),
            ]),
          ),
        ),
      );

  Widget _label(String s) => Padding(
        padding: const EdgeInsets.only(bottom: 4),
        child: Text(s, style: const TextStyle(fontSize: 11, color: Fx.dim)),
      );

  Widget _field(TextEditingController c,
          {String? hint, int lines = 1, bool mono = false}) =>
      TextField(
        controller: c,
        maxLines: lines,
        style: TextStyle(fontSize: 13, fontFamily: mono ? 'monospace' : null),
        decoration: InputDecoration(
          hintText: hint,
          isDense: true,
          filled: true,
          fillColor: Fx.surface2,
          border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none),
        ),
      );

  Widget _example(String type, String body) => Padding(
        padding: const EdgeInsets.only(top: 6),
        child: InkWell(
          onTap: () => setState(() {
            _pubType.text = type;
            _pubBody.text = const JsonEncoder.withIndent('  ')
                .convert(jsonDecode(body));
          }),
          child: Text('· $type  →  $body',
              style: const TextStyle(fontSize: 10.5, fontFamily: 'monospace', color: Fx.accent)),
        ),
      );

  // Expandable-ish flat rendering of the decoded map.
  Widget _tree(Object? v, int depth) {
    final pad = EdgeInsets.only(left: depth * 14.0);
    if (v is Map) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          for (final e in v.entries)
            Padding(
              padding: pad,
              child: (e.value is Map || e.value is List)
                  ? Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                      Text('${e.key}:',
                          style: const TextStyle(fontFamily: 'monospace', fontSize: 12.5, color: Fx.dim)),
                      _tree(e.value, depth + 1),
                    ])
                  : _kv('${e.key}', e.value),
            ),
        ],
      );
    }
    if (v is List) {
      final n = v.length;
      return Padding(
        padding: pad,
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Text('[$n]', style: const TextStyle(fontFamily: 'monospace', fontSize: 12, color: Fx.dim)),
          for (var i = 0; i < n && i < 8; i++) _tree(v[i], depth + 1),
          if (n > 8) Padding(padding: pad, child: const Text('…', style: TextStyle(color: Fx.dim))),
        ]),
      );
    }
    return _kv('', v);
  }

  Widget _kv(String k, Object? v) => Text(
        k.isEmpty ? '$v' : '$k: $v',
        style: const TextStyle(fontFamily: 'monospace', fontSize: 12.5, color: Fx.text),
      );

  Widget _statusBar() => Container(
        height: 24,
        color: Fx.rail,
        padding: const EdgeInsets.symmetric(horizontal: 12),
        child: Row(children: [
          const Icon(Icons.circle, size: 8, color: Fx.ok),
          const SizedBox(width: 6),
          Text('CycloneDDS direct · domain $_domain · ${_graph.length} topics · $_rate msg/s',
              style: const TextStyle(fontSize: 11, color: Fx.dim)),
        ]),
      );
}
