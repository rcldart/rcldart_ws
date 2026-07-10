// cdr.dart — aligned CDR (XCDR1 / "classic CDR") reader + writer.
//
// ROS 2 messages go on the wire as CDR with a 4-byte encapsulation header:
//   [0x00, repId, options_hi, options_lo]
// repId bit 0 selects endianness (1 = little). All alignment is relative to the
// start of the encapsulation BODY (the byte after the 4-byte header).
import 'dart:convert';
import 'dart:typed_data';

/// Reads primitives from an aligned CDR body.
class CdrReader {
  CdrReader(Uint8List payload)
      : _d = ByteData.sublistView(payload),
        _little = payload.length >= 2 ? (payload[1] & 0x01) == 1 : true,
        _base = payload.length >= 4 ? 4 : 0 {
    _p = _base;
  }
  final ByteData _d;
  final bool _little;
  final int _base;
  late int _p;

  Endian get _e => _little ? Endian.little : Endian.big;
  bool get little => _little;

  void _align(int n) {
    final pad = (n - ((_p - _base) % n)) % n;
    _p += pad;
  }

  int u8() => _d.getUint8(_p++);
  int i8() => _d.getInt8(_p++);
  int u16() { _align(2); final v = _d.getUint16(_p, _e); _p += 2; return v; }
  int i16() { _align(2); final v = _d.getInt16(_p, _e); _p += 2; return v; }
  int u32() { _align(4); final v = _d.getUint32(_p, _e); _p += 4; return v; }
  int i32() { _align(4); final v = _d.getInt32(_p, _e); _p += 4; return v; }
  int u64() { _align(8); final v = _d.getUint64(_p, _e); _p += 8; return v; }
  int i64() { _align(8); final v = _d.getInt64(_p, _e); _p += 8; return v; }
  double f32() { _align(4); final v = _d.getFloat32(_p, _e); _p += 4; return v; }
  double f64() { _align(8); final v = _d.getFloat64(_p, _e); _p += 8; return v; }

  String str() {
    final len = u32(); // includes the NUL terminator
    if (len == 0) return '';
    final b = Uint8List.sublistView(_d, _p, _p + len - 1);
    _p += len;
    return utf8.decode(b, allowMalformed: true);
  }

  Uint8List bytes(int n) {
    final b = Uint8List.fromList(Uint8List.sublistView(_d, _p, _p + n));
    _p += n;
    return b;
  }
}

/// Writes primitives to an aligned CDR body, growing as needed.
class CdrWriter {
  CdrWriter({bool little = true}) : _little = little {
    // Encapsulation header: CDR_LE = 00 01 00 00, CDR_BE = 00 00 00 00.
    _bytes.addAll([0x00, little ? 0x01 : 0x00, 0x00, 0x00]);
    _base = 4;
  }
  final bool _little;
  final List<int> _bytes = [];
  late int _base;
  Endian get _e => _little ? Endian.little : Endian.big;

  int get _len => _bytes.length;
  void _align(int n) {
    final pad = (n - ((_len - _base) % n)) % n;
    for (var i = 0; i < pad; i++) {
      _bytes.add(0);
    }
  }

  void _put(int n, void Function(ByteData) w) {
    _align(n);
    final bd = ByteData(n);
    w(bd);
    _bytes.addAll(bd.buffer.asUint8List());
  }

  void u8(int v) => _bytes.add(v & 0xFF);
  void i8(int v) => _bytes.add(v & 0xFF);
  void u16(int v) => _put(2, (b) => b.setUint16(0, v, _e));
  void i16(int v) => _put(2, (b) => b.setInt16(0, v, _e));
  void u32(int v) => _put(4, (b) => b.setUint32(0, v, _e));
  void i32(int v) => _put(4, (b) => b.setInt32(0, v, _e));
  void u64(int v) => _put(8, (b) => b.setUint64(0, v, _e));
  void i64(int v) => _put(8, (b) => b.setInt64(0, v, _e));
  void f32(double v) => _put(4, (b) => b.setFloat32(0, v, _e));
  void f64(double v) => _put(8, (b) => b.setFloat64(0, v, _e));

  void str(String s) {
    final b = utf8.encode(s);
    u32(b.length + 1); // + NUL
    _bytes.addAll(b);
    _bytes.add(0);
  }

  void rawBytes(List<int> b) => _bytes.addAll(b);

  /// The full CDR buffer including the 4-byte encapsulation header.
  Uint8List toBytes() => Uint8List.fromList(_bytes);
}
