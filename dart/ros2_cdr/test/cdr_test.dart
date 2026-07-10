import 'dart:typed_data';
import 'package:ros2_cdr/ros2_cdr.dart';
import 'package:test/test.dart';

void main() {
  test('std_msgs/String round-trips and matches known CDR layout', () {
    final reg = MsgRegistry()..addSingle('std_msgs/msg/String', 'string data');
    final codec = Ros2Codec(reg);

    final bytes = codec.encode('std_msgs/msg/String', {'data': 'hello'});
    // header(4) + len u32=6 (5 chars + NUL) + "hello\0"
    expect(bytes.sublist(0, 4), [0x00, 0x01, 0x00, 0x00]); // CDR_LE
    expect(bytes.sublist(4, 8), [6, 0, 0, 0]); // length incl NUL, little-endian
    expect(String.fromCharCodes(bytes.sublist(8, 13)), 'hello');
    expect(bytes[13], 0); // NUL

    final back = codec.decode('std_msgs/msg/String', bytes);
    expect(back['data'], 'hello');
  });

  test('nested Header + primitives + alignment round-trip', () {
    final reg = MsgRegistry()
      ..addConcatenated('geometry_msgs/msg/PointStamped', [
        'std_msgs/Header header',
        'geometry_msgs/Point point',
        '================================================================',
        'MSG: std_msgs/Header',
        'builtin_interfaces/Time stamp',
        'string frame_id',
        '================================================================',
        'MSG: builtin_interfaces/Time',
        'int32 sec',
        'uint32 nanosec',
        '================================================================',
        'MSG: geometry_msgs/Point',
        'float64 x',
        'float64 y',
        'float64 z',
      ].join('\n'));
    final codec = Ros2Codec(reg);

    final msg = {
      'header': {
        'stamp': {'sec': 12, 'nanosec': 345},
        'frame_id': 'map',
      },
      'point': {'x': 1.5, 'y': -2.25, 'z': 3.0},
    };
    final bytes = codec.encode('geometry_msgs/msg/PointStamped', msg);
    final back = codec.decode('geometry_msgs/msg/PointStamped', bytes);

    expect((back['header'] as Map)['stamp'], {'sec': 12, 'nanosec': 345});
    expect((back['header'] as Map)['frame_id'], 'map');
    expect((back['point'] as Map)['x'], 1.5);
    expect((back['point'] as Map)['y'], -2.25);
    expect((back['point'] as Map)['z'], 3.0);
  });

  test('sequences and fixed arrays round-trip', () {
    final reg = MsgRegistry()
      ..addSingle('sensor_msgs/msg/Toy', [
        'float64[] ranges', // unbounded sequence
        'int32[3] triple', // fixed array
        'uint8[] blob', // byte sequence -> Uint8List
      ].join('\n'));
    final codec = Ros2Codec(reg);

    final msg = {
      'ranges': [0.1, 0.2, 0.3],
      'triple': [7, 8, 9],
      'blob': Uint8List.fromList([1, 2, 3, 255]),
    };
    final bytes = codec.encode('sensor_msgs/msg/Toy', msg);
    final back = codec.decode('sensor_msgs/msg/Toy', bytes);

    expect((back['ranges'] as List).cast<double>(), [0.1, 0.2, 0.3]);
    expect((back['triple'] as List).cast<int>(), [7, 8, 9]);
    expect(back['blob'], Uint8List.fromList([1, 2, 3, 255]));
  });

  test('big-endian decode', () {
    final reg = MsgRegistry()..addSingle('T', 'int32 x');
    final codec = Ros2Codec(reg);
    // CDR_BE header + big-endian int32 = 1
    final be = Uint8List.fromList([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]);
    expect(codec.decode('T', be)['x'], 1);
  });
}
