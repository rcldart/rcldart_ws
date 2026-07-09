// flutglove — a Foxglove-like CONFIGURABLE-PANEL ROS 2 app on rcldart.
//
// Data model (mirrors Foxglove):
//  * ONE data source: a rcldart Node + a central DynamicTopicHub that
//    de-duplicates subscriptions and routes messages to panels.
//  * GENERIC decode: every message is decoded at runtime from its rosidl
//    introspection schema (via ros_cdr) into a plain Dart map — so ANY topic on
//    the graph can be bound to a panel, with NO generated Dart class required.
//  * A LAYOUT is a binary MOSAIC tree (splittable panels, not fixed tabs) plus
//    a `configById` map of per-panel settings (type + topic + field).
import 'dart:async';
import 'dart:convert';
import 'dart:io' show File, Platform;
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show MethodChannel;
import 'package:path_provider/path_provider.dart';
import 'package:rcldart/rcldart.dart' as ros;

// ---------------------------------------------------------------------------
// Layout model — a binary mosaic tree + per-panel config, keyed by panel id.
// ---------------------------------------------------------------------------
class PanelConfig {
  String type; // 'raw' | 'plot'
  String? topic;
  String? field;
  PanelConfig(this.type, {this.topic, this.field});

  Map<String, Object?> toJson() => {'type': type, 'topic': topic, 'field': field};
  factory PanelConfig.fromJson(Map<String, Object?> j) =>
      PanelConfig(j['type'] as String? ?? 'raw',
          topic: j['topic'] as String?, field: j['field'] as String?);
}

sealed class MosaicNode {
  Map<String, Object?> toJson();
  static MosaicNode fromJson(Map<String, Object?> j) => j['leaf'] != null
      ? Leaf(j['leaf'] as String)
      : Split(
          (j['axis'] as String) == 'h' ? Axis.horizontal : Axis.vertical,
          MosaicNode.fromJson((j['first'] as Map).cast()),
          MosaicNode.fromJson((j['second'] as Map).cast()),
          (j['ratio'] as num).toDouble());
}

class Leaf extends MosaicNode {
  final String id;
  Leaf(this.id);
  @override
  Map<String, Object?> toJson() => {'leaf': id};
}

class Split extends MosaicNode {
  Axis axis;
  MosaicNode first, second;
  double ratio;
  Split(this.axis, this.first, this.second, [this.ratio = 0.5]);
  @override
  Map<String, Object?> toJson() => {
        'axis': axis == Axis.horizontal ? 'h' : 'v',
        'ratio': ratio,
        'first': first.toJson(),
        'second': second.toJson(),
      };
}

int _idCounter = 0;
String _newId() => 'p${_idCounter++}';

class LayoutModel extends ChangeNotifier {
  MosaicNode root;
  final Map<String, PanelConfig> configById = {};

  LayoutModel() : root = Leaf('p0') {
    configById['p0'] = PanelConfig('raw');
    _idCounter = 1;
  }

  String split(String id, Axis axis, [String type = 'raw']) {
    final newId = _newId();
    configById[newId] = PanelConfig(type);
    root = _replace(root, id, (leaf) => Split(axis, leaf, Leaf(newId)));
    notifyListeners();
    return newId;
  }

  /// Adds a new panel of [type] beside the currently selected one (or the root)
  /// and selects it.
  void addPanel(String type) {
    final target = _selectedPanel.value ?? (root is Leaf ? (root as Leaf).id : 'p0');
    // If the target panel is empty & untyped-default, retype it in place.
    final cfg = configById[target];
    if (root is Leaf && cfg != null && cfg.topic == null) {
      cfg.type = type;
      notifyListeners();
      return;
    }
    final newId = split(target, Axis.horizontal, type);
    _selectedPanel.value = newId;
  }

  void close(String id) {
    if (root is Leaf) return; // keep at least one
    root = _remove(root, id)!;
    configById.remove(id);
    notifyListeners();
  }

  void update() => notifyListeners();

  MosaicNode _replace(MosaicNode n, String id, MosaicNode Function(Leaf) f) {
    if (n is Leaf) return n.id == id ? f(n) : n;
    final s = n as Split;
    return Split(s.axis, _replace(s.first, id, f), _replace(s.second, id, f), s.ratio);
  }

  MosaicNode? _remove(MosaicNode n, String id) {
    if (n is Leaf) return n.id == id ? null : n;
    final s = n as Split;
    final a = _remove(s.first, id), b = _remove(s.second, id);
    if (a == null) return b;
    if (b == null) return a;
    return Split(s.axis, a, b, s.ratio);
  }

  // ---- persistence -------------------------------------------------------
  Future<File> _file() async =>
      File('${(await getApplicationDocumentsDirectory()).path}/flutglove_layout.json');

  Future<void> save() async {
    final data = {
      'root': root.toJson(),
      'panels': configById.map((k, v) => MapEntry(k, v.toJson())),
    };
    await (await _file()).writeAsString(jsonEncode(data));
  }

  Future<bool> load() async {
    final f = await _file();
    if (!await f.exists()) return false;
    final data = jsonDecode(await f.readAsString()) as Map<String, Object?>;
    root = MosaicNode.fromJson((data['root'] as Map).cast());
    configById
      ..clear()
      ..addAll((data['panels'] as Map).map((k, v) =>
          MapEntry(k as String, PanelConfig.fromJson((v as Map).cast()))));
    // Continue id allocation past the highest restored panel id.
    final maxId = configById.keys
        .map((k) => int.tryParse(k.replaceAll('p', '')) ?? 0)
        .fold(0, (a, b) => a > b ? a : b);
    _idCounter = maxId + 1;
    notifyListeners();
    return true;
  }
}

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------
late ros.Node _node;
late ros.DynamicTopicHub _hub;
final _layout = LayoutModel();

const int _domainId = 0;
const List<String> _peers = <String>[];
// rmw to use on iOS/macOS — must match what tool/build_ros2_apple.sh bundled
// (RMW=fastrtps → 'rmw_fastrtps_cpp', RMW=cyclonedds → 'rmw_cyclonedds_cpp').
const String _appleRmw = 'rmw_fastrtps_cpp';

// --- Foxglove-like palette + shell state ------------------------------------
class Fx {
  static const bg = Color(0xFF0E1015);
  static const surface = Color(0xFF161922);
  static const surface2 = Color(0xFF1C2029);
  static const header = Color(0xFF1F2430);
  static const rail = Color(0xFF101219);
  static const border = Color(0xFF2A303C);
  static const accent = Color(0xFF4C82F7);
  static const accent2 = Color(0xFF8B5CF6);
  static const text = Color(0xFFE6E9EF);
  static const dim = Color(0xFF8A93A3);
  static const ok = Color(0xFF3DD68C);
}

/// The panel that sidebar topic-clicks bind to.
final ValueNotifier<String?> _selectedPanel = ValueNotifier<String?>('p0');

/// Sidebar tab: 0 = topics, 1 = layouts, -1 = collapsed.
final ValueNotifier<int> _sidebarTab = ValueNotifier<int>(0);

/// Rough message throughput counter (sampled by the status bar).
int _msgTick = 0;

class PanelType {
  final String id;
  final String label;
  final IconData icon;
  final String desc;
  const PanelType(this.id, this.label, this.icon, this.desc);
}

const _panelTypes = <PanelType>[
  PanelType('raw', 'Raw Messages', Icons.data_object, 'Message fields as a live tree'),
  PanelType('plot', 'Plot', Icons.show_chart, 'A numeric field over time'),
  PanelType('viz', 'Visualization', Icons.sensors, 'Laser · Grid · Image · Camera'),
];
IconData _iconFor(String type) =>
    _panelTypes.firstWhere((p) => p.id == type, orElse: () => _panelTypes.first).icon;

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  if (Platform.isAndroid) {
    const ch = MethodChannel('rcldart/android');
    final libDir = await ch.invokeMethod<String>('nativeLibDir');
    final ament = await ch.invokeMethod<String>('amentPrefixPath');
    ros.AndroidRosBootstrap.prepare(
        nativeLibDir: libDir, amentPrefixPath: ament, domainId: _domainId, peers: _peers);
    ros.RclDart().init();
  } else if (Platform.isIOS || Platform.isMacOS) {
    // No ROS on the device: the runtime dylibs + ament index are bundled in the
    // app (see docs/apple_ros2_architecture.md). Fetch the embedded ament path
    // from the native host and wire it before init. Falls back to a system ROS
    // (dev Mac) when no bundle is present.
    const ch = MethodChannel('rcldart/apple');
    final ament = await ch.invokeMethod<String>('amentPrefixPath');
    if (ament != null && ament.isNotEmpty) {
      ros.AppleRosBootstrap.prepare(
          amentPrefixPath: ament,
          domainId: _domainId,
          rmwImplementation: _appleRmw,
          peers: _peers);
      ros.RclDart().init();
    } else {
      ros.RclDart().init(ros.RosConfig(domainId: _domainId));
    }
  } else {
    ros.RclDart().init(
        ros.RosConfig(domainId: _domainId, rmwImplementation: 'rmw_cyclonedds_cpp'));
  }

  _node = ros.RclDart().createNode('flutglove', 'flutglove');
  _hub = ros.DynamicTopicHub(_node)..refreshGraph();
  // Drain subscriptions fast for smooth data.
  Timer.periodic(const Duration(milliseconds: 16), (_) => _hub.spinOnce());
  // Discovery keeps arriving after startup — re-scan the graph periodically so
  // topics that appear later show up in the pickers automatically.
  Timer.periodic(const Duration(milliseconds: 1200), (_) {
    _hub.refreshGraph();
    _layout.update();
  });
  runApp(const FlutgloveApp());
}

class FlutgloveApp extends StatelessWidget {
  const FlutgloveApp({super.key});
  @override
  Widget build(BuildContext context) {
    final base = ThemeData.dark(useMaterial3: true);
    return MaterialApp(
      title: 'flutglove',
      debugShowCheckedModeBanner: false,
      theme: base.copyWith(
        scaffoldBackgroundColor: Fx.bg,
        canvasColor: Fx.surface,
        colorScheme: base.colorScheme.copyWith(
          primary: Fx.accent,
          secondary: Fx.accent2,
          surface: Fx.surface,
        ),
        dividerColor: Fx.border,
        textTheme: base.textTheme.apply(bodyColor: Fx.text, displayColor: Fx.text),
        tooltipTheme: const TooltipThemeData(
          decoration: BoxDecoration(color: Fx.surface2),
          textStyle: TextStyle(color: Fx.text, fontSize: 11),
        ),
      ),
      home: const FlutgloveHome(),
    );
  }
}

class FlutgloveHome extends StatelessWidget {
  const FlutgloveHome({super.key});
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Column(children: [
        const _TopBar(),
        Container(height: 1, color: Fx.border),
        Expanded(
          child: Row(children: [
            const _SideRail(),
            ValueListenableBuilder<int>(
              valueListenable: _sidebarTab,
              builder: (_, tab, __) => tab < 0
                  ? const SizedBox.shrink()
                  : SizedBox(width: 270, child: _SidePanel(tab: tab)),
            ),
            Container(width: 1, color: Fx.border),
            Expanded(
              child: AnimatedBuilder(
                animation: _layout,
                builder: (_, __) => Padding(
                  padding: const EdgeInsets.all(3),
                  child: _buildNode(_layout.root),
                ),
              ),
            ),
          ]),
        ),
        Container(height: 1, color: Fx.border),
        const _StatusBar(),
      ]),
    );
  }

  // A split rendered as two children with a draggable divider that adjusts the
  // split ratio. LayoutBuilder gives the extent so a pixel drag maps to ratio.
  Widget _buildNode(MosaicNode n) {
    // RepaintBoundary isolates each panel's raster so a busy viz/plot doesn't
    // force siblings to repaint.
    if (n is Leaf) {
      return RepaintBoundary(child: PanelHost(key: ValueKey(n.id), id: n.id));
    }
    final s = n as Split;
    final horizontal = s.axis == Axis.horizontal;
    const grip = 6.0;
    return LayoutBuilder(builder: (context, c) {
      final extent = horizontal ? c.maxWidth : c.maxHeight;
      final firstPx = (extent - grip) * s.ratio;
      final secondPx = (extent - grip) * (1 - s.ratio);
      void onDrag(double delta) {
        final r = (s.ratio + delta / extent).clamp(0.08, 0.92);
        s.ratio = r;
        _layout.update();
      }

      final divider = GestureDetector(
        behavior: HitTestBehavior.opaque,
        onHorizontalDragUpdate:
            horizontal ? (d) => onDrag(d.delta.dx) : null,
        onVerticalDragUpdate:
            horizontal ? null : (d) => onDrag(d.delta.dy),
        child: MouseRegion(
          cursor: horizontal
              ? SystemMouseCursors.resizeColumn
              : SystemMouseCursors.resizeRow,
          child: Container(
            width: horizontal ? grip : null,
            height: horizontal ? null : grip,
            color: Fx.border,
          ),
        ),
      );

      final first = SizedBox(
          width: horizontal ? firstPx : null,
          height: horizontal ? null : firstPx,
          child: _buildNode(s.first));
      final second = SizedBox(
          width: horizontal ? secondPx : null,
          height: horizontal ? null : secondPx,
          child: _buildNode(s.second));
      return horizontal
          ? Row(children: [first, divider, second])
          : Column(children: [first, divider, second]);
    });
  }
}

// ---------------------------------------------------------------------------
// A panel: header (type + topic dropdowns + split/close) and a bound body.
// ---------------------------------------------------------------------------
class PanelHost extends StatefulWidget {
  final String id;
  const PanelHost({super.key, required this.id});
  @override
  State<PanelHost> createState() => _PanelHostState();
}

class _PanelHostState extends State<PanelHost> {
  PanelConfig get cfg => _layout.configById[widget.id]!;
  Map<String, Object?>? _last;
  final _history = <double>[];
  final _expanded = <String>{}; // Raw tree: which paths are open

  String? _bound;

  void _onMessage(Map<String, Object?> m) {
    _last = m;
    _msgTick++;
    if (cfg.type == 'plot' && cfg.field != null) {
      final v = ros.numericLeaves(m)[cfg.field!];
      if (v != null) {
        _history.add(v.toDouble());
        if (_history.length > 240) _history.removeAt(0);
      }
    }
    if (mounted) setState(() {});
  }

  // Reconciles the live subscription with cfg.topic — so binding a topic from
  // the sidebar (which mutates cfg + rebuilds) rewires this panel too.
  void _reconcile() {
    if (cfg.topic == _bound) return;
    if (_bound != null) _hub.unsubscribe(_bound!, _onMessage);
    _bound = cfg.topic;
    _last = null;
    _history.clear();
    if (_bound != null) _hub.subscribe(_bound!, _onMessage);
  }

  void _setTopic(String? topic) => setState(() => cfg.topic = topic);

  @override
  void dispose() {
    if (_bound != null) _hub.unsubscribe(_bound!, _onMessage);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    _reconcile();
    return ValueListenableBuilder<String?>(
      valueListenable: _selectedPanel,
      builder: (_, sel, __) {
        final selected = sel == widget.id;
        return GestureDetector(
          behavior: HitTestBehavior.opaque,
          onTap: () => _selectedPanel.value = widget.id,
          child: Container(
            margin: const EdgeInsets.all(2),
            decoration: BoxDecoration(
              color: Fx.surface,
              border: Border.all(
                  color: selected ? Fx.accent : Fx.border, width: selected ? 1.4 : 1),
              borderRadius: BorderRadius.circular(6),
            ),
            child: Column(children: [
              _header(),
              Container(height: 1, color: Fx.border),
              Expanded(
                child: ClipRRect(
                  borderRadius:
                      const BorderRadius.vertical(bottom: Radius.circular(5)),
                  child: _body(),
                ),
              ),
            ]),
          ),
        );
      },
    );
  }

  Widget _header() {
    return Container(
      height: 30,
      color: Fx.header,
      padding: const EdgeInsets.only(left: 2, right: 2),
      child: Row(children: [
        PopupMenuButton<String>(
          tooltip: 'panel type',
          initialValue: cfg.type,
          padding: EdgeInsets.zero,
          position: PopupMenuPosition.under,
          color: Fx.surface2,
          onSelected: (v) => setState(() {
            cfg.type = v;
            _history.clear();
          }),
          itemBuilder: (_) => [
            for (final p in _panelTypes)
              PopupMenuItem(
                value: p.id,
                child: Row(children: [
                  Icon(p.icon, size: 16, color: Fx.dim),
                  const SizedBox(width: 10),
                  Text(p.label),
                ]),
              ),
          ],
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 4),
            child: Icon(_iconFor(cfg.type), size: 16, color: Fx.accent),
          ),
        ),
        Expanded(
          child: InkWell(
            onTap: () async {
              final t = await showTopicPicker(context, cfg.topic);
              if (t != null) _setTopic(t);
            },
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 4),
              child: Row(children: [
                Flexible(
                  child: Text(cfg.topic ?? 'Select a topic…',
                      overflow: TextOverflow.ellipsis,
                      style: TextStyle(
                          fontSize: 12,
                          color: cfg.topic == null ? Fx.dim : Fx.text)),
                ),
                const Icon(Icons.expand_more, size: 15, color: Fx.dim),
              ]),
            ),
          ),
        ),
        if (cfg.type == 'plot') _fieldDropdown(),
        _hdrBtn(Icons.splitscreen, 'split right',
            () => _layout.split(widget.id, Axis.horizontal)),
        _hdrBtn(Icons.horizontal_split, 'split down',
            () => _layout.split(widget.id, Axis.vertical)),
        _hdrBtn(Icons.close, 'remove panel', () => _layout.close(widget.id)),
      ]),
    );
  }

  Widget _hdrBtn(IconData i, String tip, VoidCallback f) => IconButton(
        tooltip: tip,
        iconSize: 15,
        splashRadius: 14,
        padding: const EdgeInsets.all(4),
        constraints: const BoxConstraints(),
        color: Fx.dim,
        icon: Icon(i),
        onPressed: f,
      );

  Widget _fieldDropdown() {
    final fields = _last == null ? <String>[] : ros.numericLeaves(_last!).keys.toList();
    return DropdownButton<String>(
      value: fields.contains(cfg.field) ? cfg.field : null,
      isDense: true,
      underline: const SizedBox(),
      hint: const Text('field'),
      items: [
        for (final f in fields)
          DropdownMenuItem(value: f, child: Text(f, style: const TextStyle(fontSize: 11)))
      ],
      onChanged: (v) => setState(() {
        cfg.field = v;
        _history.clear();
      }),
    );
  }

  Widget _body() {
    if (cfg.topic == null) {
      return const Center(
          child: Text('pick a topic ↑', style: TextStyle(color: Colors.white38)));
    }
    if (cfg.type == 'plot') {
      return CustomPaint(painter: _PlotPainter(_history), child: const SizedBox.expand());
    }
    if (cfg.type == 'viz') {
      return _VizBody(_last, _hub.typeOf(cfg.topic!));
    }
    return _TreeView(_last, _hub.typeOf(cfg.topic!), _expanded, (p) {
      setState(() => _expanded.contains(p) ? _expanded.remove(p) : _expanded.add(p));
    });
  }
}

// Expandable tree of a decoded message: nested maps and arrays are collapsible;
// leaves show "key: value". Expansion state is owned by the panel (a Set of
// dotted paths) so it survives the 30ms message updates.
class _TreeView extends StatelessWidget {
  final Map<String, Object?>? msg;
  final String? type;
  final Set<String> expanded;
  final void Function(String path) onToggle;
  const _TreeView(this.msg, this.type, this.expanded, this.onToggle);

  static const _mono = TextStyle(fontFamily: 'monospace', fontSize: 12);

  @override
  Widget build(BuildContext context) {
    if (msg == null) return const Center(child: Text('waiting…'));
    return ListView(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 10),
      children: [
        Text(type ?? '', style: const TextStyle(color: Colors.tealAccent, fontSize: 12)),
        const SizedBox(height: 6),
        ..._rows(msg!, '', 0),
      ],
    );
  }

  List<Widget> _rows(Map<String, Object?> m, String prefix, int depth) {
    final out = <Widget>[];
    m.forEach((k, v) => out.addAll(_node(k, v, prefix.isEmpty ? k : '$prefix.$k', depth)));
    return out;
  }

  List<Widget> _node(String label, Object? v, String path, int depth) {
    if (v is Map<String, Object?>) {
      final open = expanded.contains(path);
      return [
        _expandRow(label, '{${v.length}}', path, depth, open),
        if (open) ..._rows(v, path, depth + 1),
      ];
    }
    if (v is List) {
      final open = expanded.contains(path);
      final rows = <Widget>[_expandRow(label, '[${v.length}]', path, depth, open)];
      if (open) {
        final n = v.length > 200 ? 200 : v.length;
        for (var i = 0; i < n; i++) {
          final e = v[i];
          if (e is Map<String, Object?>) {
            rows.addAll(_node('[$i]', e, '$path[$i]', depth + 1));
          } else {
            rows.add(_leaf('[$i]', '$e', depth + 1));
          }
        }
        if (v.length > 200) rows.add(_leaf('…', '${v.length - 200} more', depth + 1));
      }
      return rows;
    }
    return [_leaf(label, '$v', depth)];
  }

  Widget _expandRow(String label, String summary, String path, int depth, bool open) {
    return InkWell(
      onTap: () => onToggle(path),
      child: Padding(
        padding: EdgeInsets.only(left: depth * 14.0, top: 2, bottom: 2),
        child: Row(children: [
          Icon(open ? Icons.arrow_drop_down : Icons.arrow_right, size: 16),
          Text(label, style: _mono.copyWith(color: Colors.white70)),
          const SizedBox(width: 6),
          Text(summary, style: _mono.copyWith(color: Colors.white38)),
        ]),
      ),
    );
  }

  Widget _leaf(String label, String value, int depth) {
    return Padding(
      padding: EdgeInsets.only(left: depth * 14.0 + 16, top: 1, bottom: 1),
      child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
        SizedBox(width: 150, child: Text(label, style: _mono.copyWith(color: Colors.white54))),
        Expanded(child: Text(value, style: _mono)),
      ]),
    );
  }
}

class _PlotPainter extends CustomPainter {
  final List<double> data;
  _PlotPainter(this.data);
  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Offset.zero & size, Paint()..color = Colors.black26);
    if (data.length < 2) return;
    var lo = data.reduce((a, b) => a < b ? a : b);
    var hi = data.reduce((a, b) => a > b ? a : b);
    if (hi - lo < 1e-6) hi = lo + 1;
    final p = Path();
    for (var i = 0; i < data.length; i++) {
      final x = i / (data.length - 1) * size.width;
      final y = size.height - (data[i] - lo) / (hi - lo) * size.height;
      i == 0 ? p.moveTo(x, y) : p.lineTo(x, y);
    }
    canvas.drawPath(
        p,
        Paint()
          ..color = Colors.tealAccent
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2);
  }

  @override
  bool shouldRepaint(covariant _PlotPainter old) => true;
}

// Auto-visualizer: picks a renderer from the message TYPE and reads fields out
// of the generic decoded map by name — no per-type Dart class. Add a case here
// to teach flutglove a new visual.
class _VizBody extends StatelessWidget {
  final Map<String, Object?>? msg;
  final String? type;
  const _VizBody(this.msg, this.type);

  static double _d(Object? v) => (v as num?)?.toDouble() ?? 0;

  @override
  Widget build(BuildContext context) {
    if (msg == null) return const Center(child: Text('waiting…'));
    switch (type) {
      case 'sensor_msgs/msg/LaserScan':
        return CustomPaint(painter: _LaserPainter(msg!), child: const SizedBox.expand());
      case 'nav_msgs/msg/OccupancyGrid':
        return CustomPaint(painter: _GridPainter(msg!), child: const SizedBox.expand());
      case 'sensor_msgs/msg/Image':
      case 'sensor_msgs/msg/CompressedImage':
        return _ImageViz(msg!, type!);
      default:
        return Center(
          child: Text('no visualizer for\n${type ?? '?'}\n(use Raw / Plot)',
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.white38)),
        );
    }
  }
}

class _LaserPainter extends CustomPainter {
  final Map<String, Object?> m;
  _LaserPainter(this.m);
  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Offset.zero & size, Paint()..color = Colors.black45);
    final ranges = m['ranges'];
    if (ranges is! List || ranges.isEmpty) return;
    final a0 = _VizBody._d(m['angle_min']);
    final ai = _VizBody._d(m['angle_increment']);
    var rmax = _VizBody._d(m['range_max']);
    if (rmax <= 0) rmax = 12;
    final cx = size.width / 2, cy = size.height / 2;
    final scale = (size.shortestSide / 2 - 8) / rmax;
    final dot = Paint()..color = Colors.tealAccent;
    canvas.drawCircle(Offset(cx, cy), 3, Paint()..color = Colors.orangeAccent);
    for (var i = 0; i < ranges.length; i++) {
      final r = (ranges[i] as num).toDouble();
      if (r.isNaN || r.isInfinite || r <= 0 || r > rmax) continue;
      final a = a0 + ai * i;
      // ROS x-forward; screen y-down → negate.
      final x = cx + r * scale * math.cos(a);
      final y = cy - r * scale * math.sin(a);
      canvas.drawCircle(Offset(x, y), 1.4, dot);
    }
  }

  @override
  bool shouldRepaint(covariant _LaserPainter old) => true;
}

class _GridPainter extends CustomPainter {
  final Map<String, Object?> m;
  _GridPainter(this.m);
  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Offset.zero & size, Paint()..color = Colors.black54);
    final info = m['info'];
    final data = m['data'];
    if (info is! Map || data is! List || data.isEmpty) return;
    final w = (info['width'] as num?)?.toInt() ?? 0;
    final h = (info['height'] as num?)?.toInt() ?? 0;
    if (w <= 0 || h <= 0) return;
    final cell = (size.width / w).clamp(0.0, size.height / h);
    final free = Paint()..color = const Color(0xFFEDEDED);
    final occ = Paint()..color = Colors.black;
    final unk = Paint()..color = const Color(0xFF555555);
    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x++) {
        final idx = y * w + x;
        if (idx >= data.length) break;
        final v = (data[idx] as num).toInt();
        final p = v < 0 ? unk : (v >= 50 ? occ : free);
        // flip y so map is drawn with origin bottom-left like RViz
        canvas.drawRect(
            Rect.fromLTWH(x * cell, (h - 1 - y) * cell, cell + 0.5, cell + 0.5), p);
      }
    }
  }

  @override
  bool shouldRepaint(covariant _GridPainter old) => true;
}

// Camera view: decodes sensor_msgs/Image (raw encodings) or CompressedImage
// (jpeg/png) from the generic map into a ui.Image. Decoding is async and
// throttled — while one frame decodes, the newest incoming frame is queued so
// we never fall behind or decode every single message.
class _ImageViz extends StatefulWidget {
  final Map<String, Object?> msg;
  final String type;
  const _ImageViz(this.msg, this.type);
  @override
  State<_ImageViz> createState() => _ImageVizState();
}

class _ImageVizState extends State<_ImageViz> {
  ui.Image? _image;
  String _info = '';
  bool _decoding = false;
  Map<String, Object?>? _pending;

  @override
  void initState() {
    super.initState();
    _submit(widget.msg);
  }

  @override
  void didUpdateWidget(_ImageViz old) {
    super.didUpdateWidget(old);
    _submit(widget.msg);
  }

  void _submit(Map<String, Object?> m) {
    if (_decoding) {
      _pending = m; // keep only the latest
      return;
    }
    _decoding = true;
    _decode(m).then((img) {
      if (!mounted) {
        img?.dispose();
        return;
      }
      setState(() {
        _image?.dispose();
        _image = img;
      });
      _decoding = false;
      final next = _pending;
      _pending = null;
      if (next != null) _submit(next);
    });
  }

  Future<ui.Image?> _decode(Map<String, Object?> m) async {
    try {
      if (widget.type == 'sensor_msgs/msg/CompressedImage') {
        final data = m['data'];
        if (data is! List) return null;
        final bytes = data is Uint8List ? data : Uint8List.fromList(data.cast<int>());
        final codec = await ui.instantiateImageCodec(bytes);
        final frame = await codec.getNextFrame();
        _info = '${m['format']}  ${frame.image.width}x${frame.image.height}';
        return frame.image;
      }
      // raw sensor_msgs/Image
      final w = (m['width'] as num?)?.toInt() ?? 0;
      final h = (m['height'] as num?)?.toInt() ?? 0;
      final enc = '${m['encoding']}';
      final data = m['data'];
      if (w <= 0 || h <= 0 || data is! List) return null;
      final src = data is Uint8List ? data : Uint8List.fromList(data.cast<int>());
      final rgba = _toRgba(src, w, h, enc, (m['step'] as num?)?.toInt() ?? 0);
      if (rgba == null) {
        _info = 'unsupported encoding: $enc';
        return null;
      }
      final need = w * h * _bpp(enc);
      _info = src.length < need
          ? '$enc  ${w}x$h  (truncated ${src.length}/$need)'
          : '$enc  ${w}x$h';
      final completer = Completer<ui.Image>();
      ui.decodeImageFromPixels(rgba, w, h, ui.PixelFormat.rgba8888, completer.complete);
      return completer.future;
    } catch (e) {
      _info = 'decode error: $e';
      return null;
    }
  }

  static const _supported = {
    'rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8', '8UC1', 'mono16', '16UC1'
  };

  // Convert common ROS image encodings to RGBA8888. BOUNDS-SAFE: the source
  // `data` may be shorter than w*h*bpp (a large image can be truncated by the
  // decode array cap, or `step` may not match) — never read past the end;
  // missing pixels render black instead of throwing a RangeError.
  Uint8List? _toRgba(Uint8List s, int w, int h, String enc, int step) {
    if (!_supported.contains(enc)) return null;
    final bpp = _bpp(enc);
    final rowStep = step > 0 ? step : w * bpp;
    final n = s.length;
    final out = Uint8List(w * h * 4); // zero-filled → black by default
    var o = 0;
    for (var y = 0; y < h; y++) {
      var i = y * rowStep;
      for (var x = 0; x < w; x++, i += bpp) {
        int r = 0, g = 0, b = 0, a = 255;
        if (i + bpp <= n) {
          switch (enc) {
            case 'rgb8':
              r = s[i]; g = s[i + 1]; b = s[i + 2];
            case 'bgr8':
              b = s[i]; g = s[i + 1]; r = s[i + 2];
            case 'rgba8':
              r = s[i]; g = s[i + 1]; b = s[i + 2]; a = s[i + 3];
            case 'bgra8':
              b = s[i]; g = s[i + 1]; r = s[i + 2]; a = s[i + 3];
            case 'mono8':
            case '8UC1':
              r = g = b = s[i];
            case 'mono16':
            case '16UC1':
              r = g = b = s[i + 1]; // high byte as gray
          }
        }
        out[o++] = r; out[o++] = g; out[o++] = b; out[o++] = a;
      }
    }
    return out;
  }

  int _bpp(String enc) => switch (enc) {
        'rgb8' || 'bgr8' => 3,
        'rgba8' || 'bgra8' => 4,
        'mono16' || '16UC1' => 2,
        _ => 1,
      };

  @override
  void dispose() {
    _image?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final img = _image;
    return Stack(fit: StackFit.expand, children: [
      Container(color: Colors.black),
      if (img != null)
        RawImage(image: img, fit: BoxFit.contain)
      else
        const Center(child: Text('decoding…', style: TextStyle(color: Colors.white38))),
      Positioned(
        left: 6,
        bottom: 4,
        child: Text(_info,
            style: const TextStyle(fontSize: 11, color: Colors.tealAccent, backgroundColor: Colors.black54)),
      ),
    ]);
  }
}


// ===========================================================================
// Foxglove-like shell: top bar, side rail + panel, status bar, pickers.
// ===========================================================================
class _TopBar extends StatelessWidget {
  const _TopBar();
  @override
  Widget build(BuildContext context) {
    return Container(
      height: 46,
      color: Fx.rail,
      padding: const EdgeInsets.symmetric(horizontal: 10),
      child: Row(children: [
        const Icon(Icons.hub, color: Fx.accent, size: 20),
        const SizedBox(width: 8),
        const Text('flutglove',
            style: TextStyle(fontWeight: FontWeight.w600, fontSize: 15)),
        const SizedBox(width: 8),
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
          decoration: BoxDecoration(
              color: Fx.surface2, borderRadius: BorderRadius.circular(4)),
          child: const Text('rcldart · ros_cdr',
              style: TextStyle(fontSize: 10, color: Fx.dim)),
        ),
        const Spacer(),
        _TopBtn(Icons.add, 'Add a panel', label: 'Add panel', accent: true,
            onTap: () async {
          final t = await showPanelPicker(context);
          if (t != null) _layout.addPanel(t);
        }),
        const SizedBox(width: 8),
        _TopBtn(Icons.save_outlined, 'Save layout', onTap: () async {
          await _layout.save();
          if (context.mounted) {
            ScaffoldMessenger.of(context).showSnackBar(
                const SnackBar(content: Text('Layout saved')));
          }
        }),
        _TopBtn(Icons.folder_open_outlined, 'Load layout', onTap: () async {
          final ok = await _layout.load();
          if (context.mounted) {
            ScaffoldMessenger.of(context).showSnackBar(
                SnackBar(content: Text(ok ? 'Layout loaded' : 'No saved layout')));
          }
        }),
        _TopBtn(Icons.refresh, 'Refresh topic graph', onTap: () {
          _hub.refreshGraph();
          _layout.update();
        }),
      ]),
    );
  }
}

class _TopBtn extends StatelessWidget {
  final IconData icon;
  final String tip;
  final VoidCallback onTap;
  final String? label;
  final bool accent;
  const _TopBtn(this.icon, this.tip,
      {required this.onTap, this.label, this.accent = false});
  @override
  Widget build(BuildContext context) {
    return Tooltip(
      message: tip,
      child: InkWell(
        onTap: onTap,
        borderRadius: BorderRadius.circular(6),
        child: Container(
          padding: EdgeInsets.symmetric(horizontal: label == null ? 6 : 10, vertical: 6),
          decoration: accent
              ? BoxDecoration(color: Fx.accent, borderRadius: BorderRadius.circular(6))
              : null,
          child: Row(mainAxisSize: MainAxisSize.min, children: [
            Icon(icon, size: 16, color: accent ? Colors.white : Fx.dim),
            if (label != null) ...[
              const SizedBox(width: 6),
              Text(label!,
                  style: TextStyle(
                      fontSize: 12.5,
                      color: accent ? Colors.white : Fx.text,
                      fontWeight: FontWeight.w500)),
            ],
          ]),
        ),
      ),
    );
  }
}

class _SideRail extends StatelessWidget {
  const _SideRail();
  @override
  Widget build(BuildContext context) {
    return Container(
      width: 46,
      color: Fx.rail,
      child: ValueListenableBuilder<int>(
        valueListenable: _sidebarTab,
        builder: (_, tab, __) => Column(children: [
          const SizedBox(height: 6),
          _btn(Icons.list_alt, 'Topics', tab == 0,
              () => _sidebarTab.value = tab == 0 ? -1 : 0),
          _btn(Icons.dashboard_customize_outlined, 'Layouts', tab == 1,
              () => _sidebarTab.value = tab == 1 ? -1 : 1),
        ]),
      ),
    );
  }

  Widget _btn(IconData i, String tip, bool active, VoidCallback f) => Tooltip(
        message: tip,
        child: InkWell(
          onTap: f,
          child: Container(
            height: 42,
            decoration: BoxDecoration(
              border: Border(
                  left: BorderSide(
                      color: active ? Fx.accent : Colors.transparent, width: 2.5)),
              color: active ? Fx.surface : null,
            ),
            child: Icon(i, size: 20, color: active ? Fx.accent : Fx.dim),
          ),
        ),
      );
}

class _SidePanel extends StatelessWidget {
  final int tab;
  const _SidePanel({required this.tab});
  @override
  // Material (not a bare ColoredBox) so ListTile ink/splashes have a surface to
  // paint on — a Container(color:) here makes ListTile assert every frame.
  Widget build(BuildContext context) => Material(
        color: Fx.surface,
        child: tab == 0 ? const _TopicList() : const _LayoutsPanel(),
      );
}

class _TopicList extends StatefulWidget {
  const _TopicList();
  @override
  State<_TopicList> createState() => _TopicListState();
}

class _TopicListState extends State<_TopicList> {
  final _q = TextEditingController();
  Timer? _t;
  @override
  void initState() {
    super.initState();
    _t = Timer.periodic(const Duration(seconds: 1), (_) {
      if (mounted) setState(() {});
    });
  }

  @override
  void dispose() {
    _t?.cancel();
    _q.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final q = _q.text.toLowerCase();
    final topics = _hub.topics.toList()..sort();
    final items = topics.where((t) {
      if (q.isEmpty) return true;
      return t.toLowerCase().contains(q) ||
          (_hub.typeOf(t)?.toLowerCase().contains(q) ?? false);
    }).toList();
    return Column(children: [
      Padding(
        padding: const EdgeInsets.fromLTRB(12, 12, 12, 6),
        child: Row(children: [
          const Text('TOPICS',
              style: TextStyle(fontSize: 11, letterSpacing: 1, color: Fx.dim)),
          const Spacer(),
          Text('${items.length}', style: const TextStyle(fontSize: 11, color: Fx.dim)),
        ]),
      ),
      Padding(
        padding: const EdgeInsets.symmetric(horizontal: 8),
        child: TextField(
          controller: _q,
          onChanged: (_) => setState(() {}),
          style: const TextStyle(fontSize: 12),
          decoration: InputDecoration(
            isDense: true,
            hintText: 'Filter topics…',
            prefixIcon: const Icon(Icons.search, size: 16, color: Fx.dim),
            prefixIconConstraints: const BoxConstraints(minWidth: 32),
            filled: true,
            fillColor: Fx.surface2,
            contentPadding: const EdgeInsets.symmetric(vertical: 8),
            border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(6), borderSide: BorderSide.none),
          ),
        ),
      ),
      const SizedBox(height: 6),
      Expanded(
        child: ValueListenableBuilder<String?>(
          valueListenable: _selectedPanel,
          builder: (_, sel, __) {
            final boundTopic = sel == null ? null : _layout.configById[sel]?.topic;
            if (items.isEmpty) {
              return const Center(
                  child: Text('No topics yet…\nwaiting for discovery',
                      textAlign: TextAlign.center,
                      style: TextStyle(color: Fx.dim, fontSize: 12)));
            }
            return ListView.builder(
              itemCount: items.length,
              itemExtent: 42,
              itemBuilder: (_, i) {
                final t = items[i];
                final active = t == boundTopic;
                return InkWell(
                  onTap: () {
                    final id = _selectedPanel.value;
                    if (id == null) return;
                    _layout.configById[id]!.topic = t;
                    _layout.update();
                  },
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
                    color: active ? Fx.surface2 : null,
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Text(t,
                            overflow: TextOverflow.ellipsis,
                            style: TextStyle(
                                fontSize: 12.5,
                                color: active ? Fx.accent : Fx.text)),
                        Text(_hub.typeOf(t) ?? '',
                            overflow: TextOverflow.ellipsis,
                            style: const TextStyle(fontSize: 10.5, color: Fx.dim)),
                      ],
                    ),
                  ),
                );
              },
            );
          },
        ),
      ),
      Container(
        padding: const EdgeInsets.all(8),
        width: double.infinity,
        color: Fx.surface2,
        child: const Text('Click a topic to bind it to the selected panel',
            style: TextStyle(fontSize: 10.5, color: Fx.dim)),
      ),
    ]);
  }
}

class _LayoutsPanel extends StatelessWidget {
  const _LayoutsPanel();
  @override
  Widget build(BuildContext context) {
    Widget tile(IconData i, String t, String s, VoidCallback f) => ListTile(
          dense: true,
          leading: Icon(i, size: 18, color: Fx.accent),
          title: Text(t, style: const TextStyle(fontSize: 13)),
          subtitle: Text(s, style: const TextStyle(fontSize: 11, color: Fx.dim)),
          onTap: f,
        );
    return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      const Padding(
        padding: EdgeInsets.fromLTRB(12, 12, 12, 6),
        child: Text('LAYOUT',
            style: TextStyle(fontSize: 11, letterSpacing: 1, color: Fx.dim)),
      ),
      tile(Icons.save_outlined, 'Save layout', 'Store panels + bindings', () async {
        await _layout.save();
        if (context.mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
              const SnackBar(content: Text('Layout saved')));
        }
      }),
      tile(Icons.folder_open_outlined, 'Load layout', 'Restore saved layout', () async {
        final ok = await _layout.load();
        if (context.mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
              SnackBar(content: Text(ok ? 'Layout loaded' : 'No saved layout')));
        }
      }),
      const Divider(height: 20),
      for (final p in _panelTypes)
        ListTile(
          dense: true,
          leading: Icon(p.icon, size: 18, color: Fx.dim),
          title: Text('Add ${p.label}', style: const TextStyle(fontSize: 13)),
          subtitle: Text(p.desc, style: const TextStyle(fontSize: 11, color: Fx.dim)),
          onTap: () => _layout.addPanel(p.id),
        ),
    ]);
  }
}

class _StatusBar extends StatefulWidget {
  const _StatusBar();
  @override
  State<_StatusBar> createState() => _StatusBarState();
}

class _StatusBarState extends State<_StatusBar> {
  int _last = 0, _rate = 0;
  Timer? _t;
  @override
  void initState() {
    super.initState();
    _t = Timer.periodic(const Duration(seconds: 1), (_) {
      if (!mounted) return;
      setState(() {
        _rate = _msgTick - _last;
        _last = _msgTick;
      });
    });
  }

  @override
  void dispose() {
    _t?.cancel();
    super.dispose();
  }

  Widget _sep() => const Text('  ·  ', style: TextStyle(color: Fx.border, fontSize: 11));
  Widget _txt(String s, [Color c = Fx.dim]) =>
      Text(s, style: TextStyle(fontSize: 11, color: c));

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 24,
      color: Fx.rail,
      padding: const EdgeInsets.symmetric(horizontal: 10),
      child: Row(children: [
        const Icon(Icons.circle, size: 8, color: Fx.ok),
        const SizedBox(width: 5),
        _txt('Connected', Fx.ok),
        _sep(),
        _txt('domain $_domainId'),
        _sep(),
        _txt('cyclonedds'),
        _sep(),
        _txt('${_hub.graph.length} topics'),
        _sep(),
        _txt('$_rate msg/s'),
        const Spacer(),
        _txt('flutglove · rcldart FFI'),
      ]),
    );
  }
}

// ---- pickers ---------------------------------------------------------------
Future<String?> showTopicPicker(BuildContext context, String? current) =>
    showDialog<String>(
        context: context, builder: (_) => _TopicPickerDialog(current: current));

class _TopicPickerDialog extends StatefulWidget {
  final String? current;
  const _TopicPickerDialog({this.current});
  @override
  State<_TopicPickerDialog> createState() => _TopicPickerDialogState();
}

class _TopicPickerDialogState extends State<_TopicPickerDialog> {
  final _q = TextEditingController();
  @override
  Widget build(BuildContext context) {
    final q = _q.text.toLowerCase();
    final items = (_hub.topics.toList()..sort()).where((t) {
      if (q.isEmpty) return true;
      return t.toLowerCase().contains(q) ||
          (_hub.typeOf(t)?.toLowerCase().contains(q) ?? false);
    }).toList();
    return Dialog(
      backgroundColor: Fx.surface,
      child: SizedBox(
        width: 480,
        height: 520,
        child: Column(children: [
          Padding(
            padding: const EdgeInsets.all(12),
            child: TextField(
              controller: _q,
              autofocus: true,
              onChanged: (_) => setState(() {}),
              decoration: InputDecoration(
                hintText: 'Search ${_hub.graph.length} topics…',
                prefixIcon: const Icon(Icons.search, size: 18),
                filled: true,
                fillColor: Fx.surface2,
                border: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(8),
                    borderSide: BorderSide.none),
              ),
            ),
          ),
          Expanded(
            child: ListView.builder(
              itemCount: items.length,
              itemBuilder: (_, i) {
                final t = items[i];
                final sel = t == widget.current;
                return ListTile(
                  dense: true,
                  selected: sel,
                  selectedTileColor: Fx.surface2,
                  title: Text(t,
                      style: TextStyle(
                          fontSize: 13, color: sel ? Fx.accent : Fx.text)),
                  subtitle: Text(_hub.typeOf(t) ?? '',
                      style: const TextStyle(fontSize: 11, color: Fx.dim)),
                  onTap: () => Navigator.pop(context, t),
                );
              },
            ),
          ),
        ]),
      ),
    );
  }
}

Future<String?> showPanelPicker(BuildContext context) => showDialog<String>(
      context: context,
      builder: (_) => Dialog(
        backgroundColor: Fx.surface,
        child: SizedBox(
          width: 420,
          child: Column(mainAxisSize: MainAxisSize.min, children: [
            const Padding(
              padding: EdgeInsets.all(16),
              child: Text('Add a panel',
                  style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
            ),
            for (final p in _panelTypes)
              ListTile(
                leading: CircleAvatar(
                    backgroundColor: Fx.surface2,
                    child: Icon(p.icon, color: Fx.accent, size: 20)),
                title: Text(p.label),
                subtitle: Text(p.desc,
                    style: const TextStyle(fontSize: 11.5, color: Fx.dim)),
                onTap: () => Navigator.pop(context, p.id),
              ),
            const SizedBox(height: 8),
          ]),
        ),
      ),
    );
