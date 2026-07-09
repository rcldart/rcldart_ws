// ros2_viz — a Flutter ROS 2 control panel built on rcldart:
//   * visualizes /scan (LaserScan polar plot), /imu (heading), /battery_state
//   * DRIVES the robot: a joystick publishes geometry_msgs/Twist to /cmd_vel
//   * SENDS a goal: tap the plot to publish a geometry_msgs/PoseStamped
//   * topic names are editable at runtime
//   * uses the rclcpp-style Executor (executor.addNode(node).spin())
import 'dart:async';
import 'dart:ffi' hide Size; // .ref extension; hide Size (clashes with dart:ui)
import 'dart:math' as math;

import 'package:builtin_interfaces/builtin_interfaces.dart';
import 'package:diagnostic_msgs/diagnostic_msgs.dart';
import 'package:ffi/ffi.dart' show Utf8, Utf8Pointer;
import 'package:flutter/material.dart';
import 'package:geometry_msgs/geometry_msgs.dart';
import 'package:nav_msgs/nav_msgs.dart';
import 'package:rcl_interfaces/rcl_interfaces.dart';
import 'package:rcldart/rcldart.dart' as rcldart;
import 'package:sensor_msgs/sensor_msgs.dart';
import 'package:std_msgs/std_msgs.dart';
import 'package:tf2_msgs/tf2_msgs.dart';

/// Reads a rosidl_runtime_c__String view (dynamic to avoid importing every
/// package's copy of the struct type).
String _rosStr(dynamic strView) {
  final Pointer p = strView.data as Pointer;
  final int n = strView.size as int;
  if (p == nullptr || n == 0) return '';
  return p.cast<Utf8>().toDartString(length: n);
}

class ScanData {
  final List<double> ranges;
  final double angleMin, angleIncrement, rangeMax;
  const ScanData(this.ranges, this.angleMin, this.angleIncrement, this.rangeMax);
  static const empty = ScanData([], 0, 0, 1);
}

class RobotPose {
  final double x, y, yaw;
  const RobotPose(this.x, this.y, this.yaw);
}

/// A costmap/occupancy grid snapshot (world-frame).
class CostmapData {
  final double resolution, originX, originY;
  final int width, height;
  final List<int> cells; // row-major, -1 unknown, 0 free, 100 occupied
  const CostmapData(this.resolution, this.originX, this.originY, this.width,
      this.height, this.cells);
}

// --- runtime state ---
late rcldart.Node _node;
final _exec = rcldart.Executor();

final _scan = ValueNotifier<ScanData>(ScanData.empty);
final _battery = ValueNotifier<double?>(null);
final _heading = ValueNotifier<double?>(null);
final _pose = ValueNotifier<RobotPose?>(null); // /odom
final _path = ValueNotifier<List<Offset>>(const []); // /plan (world frame)
final _costmap = ValueNotifier<CostmapData?>(null); // /local_costmap/costmap
final _waypoints = ValueNotifier<List<Offset>>(const []); // route queue (world)
final _waypointMode = ValueNotifier<bool>(false);
final _maxSpeed = ValueNotifier<double>(0.5); // joystick max linear m/s
final _status = ValueNotifier<String>('starting…');

// Foxglove-like panels
class LogLine {
  final int level;
  final String name, msg;
  const LogLine(this.level, this.name, this.msg);
}

class DiagStatus {
  final int level;
  final String name, message;
  const DiagStatus(this.level, this.name, this.message);
}

class Sample {
  final double t, v;
  const Sample(this.t, this.v);
}

final _log = ValueNotifier<List<LogLine>>(const []); // /rosout
final _diag = ValueNotifier<List<DiagStatus>>(const []); // /diagnostics
final _tf = ValueNotifier<Map<String, String>>(const {}); // child -> parent
final _speedHist = <Sample>[]; // robot speed over time (Plot)
final _batteryHist = <Sample>[];
final _plotTick = ValueNotifier<int>(0);
final _t0 = DateTime.now();
double get _now =>
    DateTime.now().difference(_t0).inMilliseconds / 1000.0;

// editable topics
final _cmdVelTopic = '/cmd_vel';
final _goalTopic = '/goal_pose';

rcldart.Publisher? _cmdVelPub;
rcldart.Publisher? _goalPub;

void main() {
  // Config selectable at startup (domain / rmw / discovery peers).
  rcldart.RclDart().init(rcldart.RosConfig(
    domainId: 0,
    rmwImplementation: 'rmw_cyclonedds_cpp',
  ));
  _node = rcldart.RclDart().createNode('ros2_viz', 'viz');

  _node.createSubscriber<SensorMsgsLaserScan>(
    topic_name: '/scan',
    messageType: SensorMsgsLaserScan(),
    callback: (m) => _scan.value =
        ScanData(m.ranges, m.angleMin, m.angleIncrement, m.rangeMax),
  );
  _node.createSubscriber<SensorMsgsImu>(
    topic_name: '/imu',
    messageType: SensorMsgsImu(),
    callback: (m) {
      final q = m.data.ref.orientation;
      _heading.value = math.atan2(
          2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    },
  );
  _node.createSubscriber<SensorMsgsBatteryState>(
    topic_name: '/battery_state',
    messageType: SensorMsgsBatteryState(),
    callback: (m) {
      _battery.value = m.percentage;
      _pushSample(_batteryHist, m.percentage <= 1 ? m.percentage * 100 : m.percentage);
    },
  );
  _node.createSubscriber<RclInterfacesLog>(
    topic_name: '/rosout',
    messageType: RclInterfacesLog(),
    callback: (m) {
      final line = LogLine(m.level, m.name, m.msg);
      _log.value = [...(_log.value.length >= 200
          ? _log.value.sublist(_log.value.length - 199)
          : _log.value), line];
    },
  );
  _node.createSubscriber<DiagnosticMsgsDiagnosticArray>(
    topic_name: '/diagnostics',
    messageType: DiagnosticMsgsDiagnosticArray(),
    callback: (m) {
      final seq = m.data.ref.status;
      final out = <DiagStatus>[];
      for (var i = 0; i < seq.size; i++) {
        // wrap the element struct to reuse its string accessors
        final s = DiagnosticMsgsDiagnosticStatus()..data = (seq.data + i);
        out.add(DiagStatus(s.level, s.name, s.message));
      }
      if (out.isNotEmpty) _diag.value = out;
    },
  );
  _node.createSubscriber<NavMsgsOdometry>(
    topic_name: '/odom',
    messageType: NavMsgsOdometry(),
    callback: (m) {
      final p = m.data.ref.pose.pose;
      final q = p.orientation;
      final yaw = math.atan2(
          2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      _pose.value = RobotPose(p.position.x, p.position.y, yaw);
      // Plot: robot speed from odom twist.
      final v = m.data.ref.twist.twist.linear;
      final speed = math.sqrt(v.x * v.x + v.y * v.y);
      _pushSample(_speedHist, speed);
    },
  );
  _node.createSubscriber<NavMsgsPath>(
    topic_name: '/plan',
    messageType: NavMsgsPath(),
    callback: (m) {
      final seq = m.data.ref.poses;
      _path.value = [
        for (var i = 0; i < seq.size; i++)
          Offset(seq.data[i].pose.position.x, seq.data[i].pose.position.y),
      ];
    },
  );
  _node.createSubscriber<NavMsgsOccupancyGrid>(
    topic_name: '/local_costmap/costmap',
    messageType: NavMsgsOccupancyGrid(),
    callback: (m) {
      final info = m.data.ref.info;
      _costmap.value = CostmapData(
        info.resolution,
        info.origin.position.x,
        info.origin.position.y,
        info.width,
        info.height,
        m.value, // int8[] grid cells
      );
    },
  );

  void tfCb(Tf2MsgsTFMessage m) {
    final seq = m.data.ref.transforms;
    final edges = Map<String, String>.from(_tf.value);
    for (var i = 0; i < seq.size; i++) {
      final t = seq.data[i];
      final child = _rosStr(t.child_frame_id);
      final parent = _rosStr(t.header.frame_id);
      if (child.isNotEmpty) edges[child] = parent;
    }
    _tf.value = edges;
  }

  _node.createSubscriber<Tf2MsgsTFMessage>(
      topic_name: '/tf', messageType: Tf2MsgsTFMessage(), callback: tfCb);
  _node.createSubscriber<Tf2MsgsTFMessage>(
      topic_name: '/tf_static', messageType: Tf2MsgsTFMessage(), callback: tfCb);

  _cmdVelPub = _node.createPublisher<GeometryMsgsTwist>(
      topic_name: _cmdVelTopic, messageType: GeometryMsgsTwist());
  _goalPub = _node.createPublisher<GeometryMsgsPoseStamped>(
      topic_name: _goalTopic, messageType: GeometryMsgsPoseStamped());

  // rclcpp-style: spin the whole node.
  _exec.addNode(_node);
  _exec.spin(period: const Duration(milliseconds: 30));
  _status.value = 'connected';

  runApp(const Ros2VizApp());
}

void publishCmdVel(double linearX, double angularZ) {
  final t = GeometryMsgsTwist();
  t.data.ref.linear.x = linearX;
  t.data.ref.angular.z = angularZ;
  _cmdVelPub?.publish(t);
}

void publishGoal(double x, double y) {
  final g = GeometryMsgsPoseStamped();
  final now = rcldart.Clock.systemNow();
  g.data.ref.header.stamp.sec = now.sec;
  g.data.ref.header.stamp.nanosec = now.nanosec;
  g.data.ref.pose.position.x = x;
  g.data.ref.pose.position.y = y;
  g.data.ref.pose.orientation.w = 1.0; // identity heading
  _goalPub?.publish(g);
  _status.value = 'goal sent (${x.toStringAsFixed(1)}, ${y.toStringAsFixed(1)})';
}

/// A tap on a map view: either queue a waypoint or send a single goal.
void onMapTap(double x, double y) {
  if (_waypointMode.value) {
    _waypoints.value = [..._waypoints.value, Offset(x, y)];
    _status.value = '${_waypoints.value.length} waypoint(s) queued';
  } else {
    publishGoal(x, y);
  }
}

void _pushSample(List<Sample> hist, double v) {
  hist.add(Sample(_now, v));
  final cutoff = _now - 30; // keep 30 s
  hist.removeWhere((s) => s.t < cutoff);
  _plotTick.value++;
}

/// Sends the queued waypoints as a sequential route (each after a delay).
Future<void> sendRoute() async {
  final wps = _waypoints.value;
  if (wps.isEmpty) return;
  for (var i = 0; i < wps.length; i++) {
    publishGoal(wps[i].dx, wps[i].dy);
    _status.value = 'route: waypoint ${i + 1}/${wps.length}';
    await Future.delayed(const Duration(seconds: 3));
  }
  _status.value = 'route complete';
}

class Ros2VizApp extends StatelessWidget {
  const Ros2VizApp({super.key});
  @override
  Widget build(BuildContext context) => MaterialApp(
        title: 'ros2_viz',
        theme: ThemeData.dark(useMaterial3: true),
        home: const VizPage(),
      );
}

class VizPage extends StatelessWidget {
  const VizPage({super.key});
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('rcldart • ROS 2 control panel'),
        actions: [
          Center(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: ValueListenableBuilder<String>(
                valueListenable: _status,
                builder: (_, s, __) => Text(s),
              ),
            ),
          ),
        ],
      ),
      body: Row(
        children: [
          Expanded(
            flex: 3,
            child: DefaultTabController(
              length: 9,
              child: Column(
                children: [
                  const TabBar(isScrollable: true, tabs: [
                    Tab(text: '3D / Map'),
                    Tab(text: 'Laser'),
                    Tab(text: 'Plot'),
                    Tab(text: 'Gauges'),
                    Tab(text: 'Log'),
                    Tab(text: 'Diagnostics'),
                    Tab(text: 'TF Tree'),
                    Tab(text: 'Topics'),
                    Tab(text: 'Raw'),
                  ]),
                  Expanded(
                    child: TabBarView(children: [
                      _MapView(),
                      _LaserView(),
                      _PlotPanel(),
                      _GaugePanel(),
                      _LogPanel(),
                      _DiagnosticsPanel(),
                      _TfPanel(),
                      _TopicsPanel(),
                      _RawPanel(),
                    ]),
                  ),
                ],
              ),
            ),
          ),
          const VerticalDivider(width: 1),
          SizedBox(width: 280, child: _ControlPanel()),
        ],
      ),
    );
  }
}

/// World-frame map: the planned route (/plan) + the robot pose (/odom),
/// auto-fitted. Tap to send a goal at that world point.
class _MapView extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16),
      child: AnimatedBuilder(
        animation: Listenable.merge([_path, _pose, _costmap, _waypoints]),
        builder: (_, __) => LayoutBuilder(builder: (_, box) {
          final size = Size(box.maxWidth, box.maxHeight);
          final map = _MapPainter(
              _path.value, _pose.value, _costmap.value, _waypoints.value);
          return GestureDetector(
            onTapUp: (d) {
              final w = map.screenToWorld(d.localPosition, size);
              if (w != null) onMapTap(w.dx, w.dy);
            },
            child: CustomPaint(painter: map, child: const SizedBox.expand()),
          );
        }),
      ),
    );
  }
}

class _MapPainter extends CustomPainter {
  final List<Offset> path;
  final RobotPose? pose;
  final CostmapData? costmap;
  final List<Offset> waypoints;
  Rect _bounds = Rect.zero;
  double _scale = 1;
  Offset _origin = Offset.zero;
  _MapPainter(this.path, this.pose, this.costmap, this.waypoints);

  void _computeFit(Size size) {
    final cm = costmap;
    final pts = [
      ...path,
      ...waypoints,
      if (pose != null) Offset(pose!.x, pose!.y),
      if (cm != null) Offset(cm.originX, cm.originY),
      if (cm != null)
        Offset(cm.originX + cm.width * cm.resolution,
            cm.originY + cm.height * cm.resolution),
    ];
    if (pts.isEmpty) {
      _bounds = const Rect.fromLTRB(-5, -5, 5, 5);
    } else {
      var minX = pts.first.dx, maxX = pts.first.dx;
      var minY = pts.first.dy, maxY = pts.first.dy;
      for (final p in pts) {
        minX = math.min(minX, p.dx);
        maxX = math.max(maxX, p.dx);
        minY = math.min(minY, p.dy);
        maxY = math.max(maxY, p.dy);
      }
      _bounds = Rect.fromLTRB(minX - 1, minY - 1, maxX + 1, maxY + 1);
    }
    final sx = size.width / _bounds.width, sy = size.height / _bounds.height;
    _scale = math.min(sx, sy);
    _origin = Offset(size.width / 2, size.height / 2);
  }

  Offset _worldToScreen(double x, double y) {
    final c = _bounds.center;
    // world x forward -> screen up; world y left -> screen left.
    return _origin + Offset(-(y - c.dy) * _scale, -(x - c.dx) * _scale);
  }

  Offset? screenToWorld(Offset s, Size size) {
    _computeFit(size);
    final v = (s - _origin) / _scale;
    final c = _bounds.center;
    return Offset(-v.dy + c.dx, -v.dx + c.dy); // invert _worldToScreen
  }

  @override
  void paint(Canvas canvas, Size size) {
    _computeFit(size);

    // Costmap: occupied cells (world-frame, using origin + resolution).
    final cm = costmap;
    if (cm != null && cm.cells.length >= cm.width * cm.height) {
      final cell = cm.resolution * _scale;
      final sz = Size(cell + 1, cell + 1);
      for (var r = 0; r < cm.height; r++) {
        for (var c = 0; c < cm.width; c++) {
          final v = cm.cells[r * cm.width + c];
          if (v <= 0) continue; // free/unknown -> skip
          final wx = cm.originX + (c + 0.5) * cm.resolution;
          final wy = cm.originY + (r + 0.5) * cm.resolution;
          final s = _worldToScreen(wx, wy);
          final alpha = (v.clamp(0, 100) / 100 * 200 + 40).toInt();
          canvas.drawRect(Rect.fromCenter(center: s, width: sz.width, height: sz.height),
              Paint()..color = Color.fromARGB(alpha, 130, 90, 200));
        }
      }
    }

    // Planned route (/plan).
    if (path.length > 1) {
      final p = Path();
      final s0 = _worldToScreen(path[0].dx, path[0].dy);
      p.moveTo(s0.dx, s0.dy);
      for (final pt in path.skip(1)) {
        final s = _worldToScreen(pt.dx, pt.dy);
        p.lineTo(s.dx, s.dy);
      }
      canvas.drawPath(
          p,
          Paint()
            ..color = Colors.tealAccent
            ..style = PaintingStyle.stroke
            ..strokeWidth = 2);
    }

    // Queued waypoints.
    for (var i = 0; i < waypoints.length; i++) {
      final s = _worldToScreen(waypoints[i].dx, waypoints[i].dy);
      canvas.drawCircle(s, 6, Paint()..color = Colors.yellowAccent);
      final tp = TextPainter(
        text: TextSpan(
            text: '${i + 1}',
            style: const TextStyle(color: Colors.black, fontSize: 10)),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, s - Offset(tp.width / 2, tp.height / 2));
    }

    // Robot pose (/odom).
    if (pose != null) {
      final o = _worldToScreen(pose!.x, pose!.y);
      canvas.drawCircle(o, 6, Paint()..color = Colors.orangeAccent);
      final tip = o + Offset(-14 * math.sin(pose!.yaw), -14 * math.cos(pose!.yaw));
      canvas.drawLine(o, tip,
          Paint()..color = Colors.orangeAccent..strokeWidth = 3);
    }
  }

  @override
  bool shouldRepaint(covariant _MapPainter old) =>
      old.path != path ||
      old.pose != pose ||
      old.costmap != costmap ||
      old.waypoints != waypoints;
}

/// Laser plot; tapping publishes a navigation goal at that point.
class _LaserView extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16),
      child: ValueListenableBuilder<ScanData>(
        valueListenable: _scan,
        builder: (_, scan, __) => LayoutBuilder(builder: (_, box) {
          final size = Size(box.maxWidth, box.maxHeight);
          return GestureDetector(
            onTapUp: (d) {
              // screen tap -> metric point in the robot frame (forward = up).
              final c = size.center(Offset.zero);
              final radius = size.shortestSide / 2 - 8;
              final maxR = scan.rangeMax > 0 ? scan.rangeMax : 10.0;
              final dx = (d.localPosition.dx - c.dx) / radius * maxR;
              final dy = (d.localPosition.dy - c.dy) / radius * maxR;
              onMapTap(-dy, -dx); // up = +x forward, left = +y
            },
            child: CustomPaint(painter: LaserPainter(scan), child: const SizedBox.expand()),
          );
        }),
      ),
    );
  }
}

class _ControlPanel extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        const Text('Drive (/cmd_vel)',
            style: TextStyle(fontWeight: FontWeight.bold)),
        const SizedBox(height: 8),
        Center(child: Joystick(onChanged: publishCmdVel)),
        const SizedBox(height: 8),
        ValueListenableBuilder<double>(
          valueListenable: _maxSpeed,
          builder: (_, v, __) => Row(children: [
            const Text('max', style: TextStyle(fontSize: 11)),
            Expanded(
              child: Slider(
                value: v,
                min: 0.1,
                max: 1.5,
                onChanged: (x) => _maxSpeed.value = x,
              ),
            ),
            Text('${v.toStringAsFixed(1)} m/s', style: const TextStyle(fontSize: 11)),
          ]),
        ),
        const Text('drag to move • release to stop • tap a view to set a goal',
            style: TextStyle(fontSize: 11, color: Colors.white54)),
        const Divider(height: 24),
        const Text('Route (waypoints)',
            style: TextStyle(fontWeight: FontWeight.bold)),
        ValueListenableBuilder<bool>(
          valueListenable: _waypointMode,
          builder: (_, on, __) => ValueListenableBuilder<List<Offset>>(
            valueListenable: _waypoints,
            builder: (_, wps, __) => Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                SwitchListTile(
                  dense: true,
                  contentPadding: EdgeInsets.zero,
                  title: const Text('tap adds waypoint'),
                  value: on,
                  onChanged: (v) => _waypointMode.value = v,
                ),
                Text('${wps.length} waypoint(s) queued',
                    style: const TextStyle(fontSize: 12)),
                Row(children: [
                  FilledButton.icon(
                    onPressed: wps.isEmpty ? null : sendRoute,
                    icon: const Icon(Icons.route, size: 16),
                    label: const Text('Send route'),
                  ),
                  const SizedBox(width: 8),
                  OutlinedButton(
                    onPressed: wps.isEmpty
                        ? null
                        : () => _waypoints.value = const [],
                    child: const Text('Clear'),
                  ),
                ]),
              ],
            ),
          ),
        ),
        const Divider(height: 32),
        const Text('/battery_state',
            style: TextStyle(fontWeight: FontWeight.bold)),
        ValueListenableBuilder<double?>(
          valueListenable: _battery,
          builder: (_, p, __) {
            if (p == null) return const Text('waiting…');
            final v = p <= 1.0 ? p : p / 100;
            return Row(children: [
              Expanded(child: LinearProgressIndicator(value: v.clamp(0, 1))),
              const SizedBox(width: 8),
              Text('${(v * 100).toStringAsFixed(0)}%'),
            ]);
          },
        ),
        const SizedBox(height: 16),
        const Text('/imu heading',
            style: TextStyle(fontWeight: FontWeight.bold)),
        ValueListenableBuilder<double?>(
          valueListenable: _heading,
          builder: (_, yaw, __) => yaw == null
              ? const Text('waiting…')
              : Text('${(yaw * 180 / math.pi).toStringAsFixed(0)}°'),
        ),
        const Divider(height: 32),
        const Text('Topics', style: TextStyle(fontWeight: FontWeight.bold)),
        const SizedBox(height: 4),
        _TopicField(label: 'cmd_vel', initial: _cmdVelTopic, onApply: (t) {
          _cmdVelPub = _node.createPublisher<GeometryMsgsTwist>(
              topic_name: t, messageType: GeometryMsgsTwist());
          _status.value = 'cmd_vel → $t';
        }),
        _TopicField(label: 'goal', initial: _goalTopic, onApply: (t) {
          _goalPub = _node.createPublisher<GeometryMsgsPoseStamped>(
              topic_name: t, messageType: GeometryMsgsPoseStamped());
          _status.value = 'goal → $t';
        }),
      ],
    );
  }
}

class _TopicField extends StatefulWidget {
  final String label, initial;
  final void Function(String) onApply;
  const _TopicField(
      {required this.label, required this.initial, required this.onApply});
  @override
  State<_TopicField> createState() => _TopicFieldState();
}

class _TopicFieldState extends State<_TopicField> {
  late final _c = TextEditingController(text: widget.initial);
  @override
  Widget build(BuildContext context) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 4),
        child: TextField(
          controller: _c,
          decoration: InputDecoration(
            isDense: true,
            labelText: widget.label,
            border: const OutlineInputBorder(),
            suffixIcon: IconButton(
                icon: const Icon(Icons.check, size: 18),
                onPressed: () => widget.onApply(_c.text)),
          ),
        ),
      );
}

/// A drag joystick that reports (linearX forward, angularZ turn) in [-1, 1].
class Joystick extends StatefulWidget {
  final void Function(double linearX, double angularZ) onChanged;
  const Joystick({super.key, required this.onChanged});
  @override
  State<Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<Joystick> {
  Offset _knob = Offset.zero;
  static const _size = 160.0;
  static const _maxAng = 1.0;

  void _update(Offset local) {
    final c = const Offset(_size / 2, _size / 2);
    var v = local - c;
    final r = _size / 2 - 16;
    if (v.distance > r) v = v * (r / v.distance);
    setState(() => _knob = v);
    widget.onChanged(-v.dy / r * _maxSpeed.value, -v.dx / r * _maxAng);
  }

  void _release() {
    setState(() => _knob = Offset.zero);
    widget.onChanged(0, 0);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanStart: (d) => _update(d.localPosition),
      onPanUpdate: (d) => _update(d.localPosition),
      onPanEnd: (_) => _release(),
      onPanCancel: _release,
      child: CustomPaint(
        size: const Size(_size, _size),
        painter: _JoystickPainter(_knob),
      ),
    );
  }
}

class _JoystickPainter extends CustomPainter {
  final Offset knob;
  _JoystickPainter(this.knob);
  @override
  void paint(Canvas canvas, Size size) {
    final c = size.center(Offset.zero);
    canvas.drawCircle(c, size.width / 2 - 2,
        Paint()..color = Colors.white10);
    canvas.drawCircle(c, size.width / 2 - 2,
        Paint()..color = Colors.white24..style = PaintingStyle.stroke);
    canvas.drawCircle(c + knob, 18, Paint()..color = Colors.tealAccent);
  }

  @override
  bool shouldRepaint(covariant _JoystickPainter old) => old.knob != knob;
}

class LaserPainter extends CustomPainter {
  final ScanData scan;
  LaserPainter(this.scan);
  @override
  void paint(Canvas canvas, Size size) {
    final center = size.center(Offset.zero);
    final radius = size.shortestSide / 2 - 8;
    final maxR = scan.rangeMax > 0 ? scan.rangeMax : 1.0;
    final grid = Paint()
      ..color = Colors.white24
      ..style = PaintingStyle.stroke;
    for (var i = 1; i <= 4; i++) {
      canvas.drawCircle(center, radius * i / 4, grid);
    }
    canvas.drawCircle(center, 4, Paint()..color = Colors.orangeAccent);
    final pt = Paint()..color = Colors.tealAccent;
    for (var i = 0; i < scan.ranges.length; i++) {
      final r = scan.ranges[i];
      if (r.isNaN || r.isInfinite || r <= 0 || r > maxR) continue;
      final a = scan.angleMin + i * scan.angleIncrement;
      final rr = r / maxR * radius;
      canvas.drawCircle(
          center + Offset(-rr * math.sin(a), -rr * math.cos(a)), 1.6, pt);
    }
  }

  @override
  bool shouldRepaint(covariant LaserPainter old) => old.scan != scan;
}

// ---------------------------------------------------------------------------
// Foxglove-like panels
// ---------------------------------------------------------------------------

/// Plot panel: robot speed (/odom) + battery (/battery_state) over the last 30s.
class _PlotPanel extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16),
      child: ValueListenableBuilder<int>(
        valueListenable: _plotTick,
        builder: (_, __, ___) => Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('speed (m/s) — teal   •   battery (%) — orange',
                style: TextStyle(fontSize: 12, color: Colors.white70)),
            const SizedBox(height: 8),
            Expanded(
              child: CustomPaint(
                painter: _PlotPainter(),
                child: const SizedBox.expand(),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _PlotPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final grid = Paint()..color = Colors.white12;
    for (var i = 0; i <= 4; i++) {
      final y = size.height * i / 4;
      canvas.drawLine(Offset(0, y), Offset(size.width, y), grid);
    }
    _series(canvas, size, _speedHist, Colors.tealAccent, 0, 2.0);
    _series(canvas, size, _batteryHist, Colors.orangeAccent, 0, 100);
  }

  void _series(Canvas canvas, Size size, List<Sample> h, Color color,
      double vmin, double vmax) {
    if (h.length < 2) return;
    final tmax = _now, tmin = _now - 30;
    final p = Path();
    for (var i = 0; i < h.length; i++) {
      final x = ((h[i].t - tmin) / (tmax - tmin)).clamp(0.0, 1.0) * size.width;
      final y = size.height -
          ((h[i].v - vmin) / (vmax - vmin)).clamp(0.0, 1.0) * size.height;
      i == 0 ? p.moveTo(x, y) : p.lineTo(x, y);
    }
    canvas.drawPath(
        p, Paint()..color = color..style = PaintingStyle.stroke..strokeWidth = 2);
  }

  @override
  bool shouldRepaint(covariant _PlotPainter old) => true;
}

const _logLevels = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'};
Color _levelColor(int l) => l >= 40
    ? Colors.redAccent
    : l == 30
        ? Colors.orangeAccent
        : l <= 10
            ? Colors.white38
            : Colors.white70;

/// Log panel: /rosout (rcl_interfaces/Log).
class _LogPanel extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<List<LogLine>>(
      valueListenable: _log,
      builder: (_, lines, __) => ListView.builder(
        reverse: true,
        padding: const EdgeInsets.all(8),
        itemCount: lines.length,
        itemBuilder: (_, i) {
          final l = lines[lines.length - 1 - i];
          return Padding(
            padding: const EdgeInsets.symmetric(vertical: 1),
            child: Text.rich(TextSpan(children: [
              TextSpan(
                  text: '${_logLevels[l.level] ?? l.level} ',
                  style: TextStyle(
                      color: _levelColor(l.level),
                      fontWeight: FontWeight.bold,
                      fontFamily: 'monospace')),
              TextSpan(
                  text: '[${l.name}] ',
                  style: const TextStyle(
                      color: Colors.white38, fontFamily: 'monospace')),
              TextSpan(
                  text: l.msg,
                  style: const TextStyle(fontFamily: 'monospace', fontSize: 12)),
            ])),
          );
        },
      ),
    );
  }
}

/// Diagnostics panel: /diagnostics (diagnostic_msgs/DiagnosticArray).
class _DiagnosticsPanel extends StatelessWidget {
  static const _dLevel = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'};
  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<List<DiagStatus>>(
      valueListenable: _diag,
      builder: (_, items, __) => items.isEmpty
          ? const Center(child: Text('no /diagnostics yet'))
          : ListView.separated(
              padding: const EdgeInsets.all(8),
              itemCount: items.length,
              separatorBuilder: (_, __) => const Divider(height: 1),
              itemBuilder: (_, i) {
                final d = items[i];
                final c = d.level == 0
                    ? Colors.greenAccent
                    : d.level == 1
                        ? Colors.orangeAccent
                        : Colors.redAccent;
                return ListTile(
                  dense: true,
                  leading: Icon(Icons.circle, color: c, size: 12),
                  title: Text(d.name, style: const TextStyle(fontSize: 13)),
                  subtitle: Text(d.message,
                      style: const TextStyle(fontSize: 11, color: Colors.white54)),
                  trailing: Text(_dLevel[d.level] ?? '${d.level}',
                      style: TextStyle(color: c, fontSize: 11)),
                );
              },
            ),
    );
  }
}

/// Raw Messages panel: latest values of the subscribed topics.
class _RawPanel extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: Listenable.merge([_scan, _pose, _battery, _heading, _costmap]),
      builder: (_, __) {
        final s = _scan.value, p = _pose.value, cm = _costmap.value;
        Widget row(String k, String v) => Padding(
              padding: const EdgeInsets.symmetric(vertical: 3),
              child: Row(children: [
                SizedBox(
                    width: 220,
                    child: Text(k,
                        style: const TextStyle(
                            fontFamily: 'monospace', color: Colors.white54))),
                Expanded(
                    child: Text(v,
                        style: const TextStyle(fontFamily: 'monospace'))),
              ]),
            );
        return ListView(padding: const EdgeInsets.all(16), children: [
          row('/scan.ranges.length', '${s.ranges.length}'),
          row('/scan.range_max', s.rangeMax.toStringAsFixed(2)),
          row('/odom.pose', p == null
              ? '—'
              : 'x=${p.x.toStringAsFixed(2)} y=${p.y.toStringAsFixed(2)} yaw=${(p.yaw * 180 / math.pi).toStringAsFixed(0)}°'),
          row('/battery_state.percentage',
              _battery.value?.toStringAsFixed(3) ?? '—'),
          row('/imu.heading',
              _heading.value == null ? '—' : '${(_heading.value! * 180 / math.pi).toStringAsFixed(0)}°'),
          row('/local_costmap', cm == null ? '—' : '${cm.width}x${cm.height} @ ${cm.resolution}m'),
        ]);
      },
    );
  }
}

/// TF Tree panel: the /tf + /tf_static frame hierarchy.
class _TfPanel extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<Map<String, String>>(
      valueListenable: _tf,
      builder: (_, edges, __) {
        if (edges.isEmpty) return const Center(child: Text('no /tf yet'));
        // roots = frames that are never a child
        final children = <String, List<String>>{};
        edges.forEach((c, p) => children.putIfAbsent(p, () => []).add(c));
        final allChildren = edges.keys.toSet();
        final roots = children.keys.where((f) => !allChildren.contains(f)).toList();
        List<Widget> build(String frame, int depth) {
          return [
            Padding(
              padding: EdgeInsets.only(left: depth * 20.0, top: 2, bottom: 2),
              child: Row(children: [
                const Icon(Icons.account_tree, size: 14, color: Colors.tealAccent),
                const SizedBox(width: 6),
                Text(frame, style: const TextStyle(fontFamily: 'monospace')),
              ]),
            ),
            for (final c in (children[frame] ?? [])) ...build(c, depth + 1),
          ];
        }
        return ListView(
          padding: const EdgeInsets.all(16),
          children: [for (final r in roots) ...build(r, 0)],
        );
      },
    );
  }
}

/// A single circular gauge.
class _Gauge extends StatelessWidget {
  final String label;
  final double? value, min, max;
  final String unit;
  const _Gauge(this.label, this.value, this.min, this.max, this.unit);
  @override
  Widget build(BuildContext context) {
    return Column(children: [
      SizedBox(
        width: 120,
        height: 120,
        child: CustomPaint(painter: _GaugePainter(value, min!, max!)),
      ),
      Text(label),
      Text(value == null ? '—' : '${value!.toStringAsFixed(1)} $unit',
          style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
    ]);
  }
}

class _GaugePainter extends CustomPainter {
  final double? value;
  final double min, max;
  _GaugePainter(this.value, this.min, this.max);
  @override
  void paint(Canvas canvas, Size size) {
    final c = size.center(Offset.zero);
    final r = size.shortestSide / 2 - 8;
    const start = math.pi * 0.75, sweep = math.pi * 1.5;
    canvas.drawArc(Rect.fromCircle(center: c, radius: r), start, sweep, false,
        Paint()..color = Colors.white12..style = PaintingStyle.stroke..strokeWidth = 10);
    if (value == null) return;
    final f = ((value! - min) / (max - min)).clamp(0.0, 1.0);
    final col = f > 0.66 ? Colors.greenAccent : (f > 0.33 ? Colors.orangeAccent : Colors.redAccent);
    canvas.drawArc(Rect.fromCircle(center: c, radius: r), start, sweep * f, false,
        Paint()..color = col..style = PaintingStyle.stroke..strokeWidth = 10..strokeCap = StrokeCap.round);
  }

  @override
  bool shouldRepaint(covariant _GaugePainter old) => old.value != value;
}

/// Gauge/Indicator panel: battery + speed.
class _GaugePanel extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: Listenable.merge([_battery, _plotTick]),
      builder: (_, __) {
        final b = _battery.value;
        final bp = b == null ? null : (b <= 1 ? b * 100 : b);
        final speed = _speedHist.isEmpty ? 0.0 : _speedHist.last.v;
        return Center(
          child: Wrap(
            spacing: 32,
            runSpacing: 16,
            alignment: WrapAlignment.center,
            children: [
              _Gauge('Battery', bp, 0, 100, '%'),
              _Gauge('Speed', speed, 0, 2, 'm/s'),
            ],
          ),
        );
      },
    );
  }
}

/// Topics browser (Foxglove "topic list"): the live ROS graph — every topic
/// and its type, discovered via graph introspection. This is how panels learn
/// which topics exist to bind to.
class _TopicsPanel extends StatefulWidget {
  @override
  State<_TopicsPanel> createState() => _TopicsPanelState();
}

class _TopicsPanelState extends State<_TopicsPanel> {
  Map<String, List<String>> _topics = {};
  String _filter = '';

  @override
  void initState() {
    super.initState();
    _refresh();
  }

  void _refresh() {
    try {
      setState(() => _topics = _node.getTopicNamesAndTypes());
    } catch (_) {}
  }

  @override
  Widget build(BuildContext context) {
    final entries = _topics.entries
        .where((e) => _filter.isEmpty || e.key.contains(_filter))
        .toList()
      ..sort((a, b) => a.key.compareTo(b.key));
    return Column(children: [
      Padding(
        padding: const EdgeInsets.all(8),
        child: Row(children: [
          Expanded(
            child: TextField(
              decoration: const InputDecoration(
                  isDense: true,
                  prefixIcon: Icon(Icons.search, size: 18),
                  hintText: 'filter topics',
                  border: OutlineInputBorder()),
              onChanged: (v) => setState(() => _filter = v),
            ),
          ),
          const SizedBox(width: 8),
          Text('${entries.length}/${_topics.length}'),
          IconButton(icon: const Icon(Icons.refresh), onPressed: _refresh),
        ]),
      ),
      Expanded(
        child: ListView.builder(
          itemCount: entries.length,
          itemBuilder: (_, i) => ListTile(
            dense: true,
            title: Text(entries[i].key,
                style: const TextStyle(fontFamily: 'monospace', fontSize: 13)),
            subtitle: Text(entries[i].value.join(', '),
                style: const TextStyle(fontSize: 11, color: Colors.tealAccent)),
          ),
        ),
      ),
    ]);
  }
}
