// codec.dart — schema-driven ROS 2 message <-> Dart map, over CDR.
//
// decode(): CDR bytes + type -> Map<String,Object?>
// encode(): Map<String,Object?> + type -> CDR bytes
//
// Both walk the [MsgRegistry] recursively, so ANY message decodes/encodes with
// no generated code — the whole point of the dependency-minimal design.
import 'dart:typed_data';

import 'cdr.dart';
import 'msg_def.dart';

class Ros2Codec {
  Ros2Codec(this.registry);
  final MsgRegistry registry;

  // ---- decode ---------------------------------------------------------------
  Map<String, Object?> decode(String type, Uint8List cdr) {
    final def = registry[canonicalType(type)];
    if (def == null) return {'_no_schema': type};
    return _readStruct(def, CdrReader(cdr));
  }

  Map<String, Object?> _readStruct(MsgDef def, CdrReader r) {
    final out = <String, Object?>{};
    for (final f in def.fields) {
      out[f.name] = f.isArray ? _readArray(f, r) : _readOne(f, r);
    }
    return out;
  }

  Object? _readArray(Field f, CdrReader r) {
    final len = f.fixedLen ?? r.u32();
    if (f.isPrimitive && (f.type == 'uint8' || f.type == 'byte' || f.type == 'char')) {
      return r.bytes(len);
    }
    final list = List<Object?>.filled(len, null, growable: false);
    for (var i = 0; i < len; i++) {
      list[i] = _readOne(f, r);
    }
    return list;
  }

  Object? _readOne(Field f, CdrReader r) {
    if (!f.isPrimitive) {
      final def = registry[f.type];
      return def == null ? {'_no_schema': f.type} : _readStruct(def, r);
    }
    switch (f.type) {
      case 'bool': return r.u8() != 0;
      case 'byte': case 'uint8': case 'char': return r.u8();
      case 'int8': return r.i8();
      case 'int16': return r.i16();
      case 'uint16': return r.u16();
      case 'int32': return r.i32();
      case 'uint32': return r.u32();
      case 'int64': return r.i64();
      case 'uint64': return r.u64();
      case 'float32': return r.f32();
      case 'float64': return r.f64();
      case 'string': case 'wstring': return r.str();
      default: return null;
    }
  }

  // ---- encode ---------------------------------------------------------------
  Uint8List encode(String type, Map<String, Object?> msg, {bool little = true}) {
    final def = registry[canonicalType(type)];
    if (def == null) {
      throw StateError('no schema registered for $type');
    }
    final w = CdrWriter(little: little);
    _writeStruct(def, msg, w);
    return w.toBytes();
  }

  void _writeStruct(MsgDef def, Map<String, Object?> msg, CdrWriter w) {
    for (final f in def.fields) {
      final v = msg[f.name];
      if (f.isArray) {
        _writeArray(f, v, w);
      } else {
        _writeOne(f, v, w);
      }
    }
  }

  void _writeArray(Field f, Object? v, CdrWriter w) {
    if (f.isPrimitive && (f.type == 'uint8' || f.type == 'byte' || f.type == 'char')) {
      final b = v is Uint8List ? v : Uint8List.fromList((v as List?)?.cast<int>() ?? const []);
      if (f.fixedLen == null) w.u32(b.length);
      w.rawBytes(b);
      return;
    }
    final list = (v as List?) ?? const [];
    if (f.fixedLen == null) w.u32(list.length);
    for (final e in list) {
      _writeOne(f, e, w);
    }
  }

  void _writeOne(Field f, Object? v, CdrWriter w) {
    if (!f.isPrimitive) {
      final def = registry[f.type];
      if (def == null) throw StateError('no schema for nested ${f.type}');
      _writeStruct(def, (v as Map?)?.cast<String, Object?>() ?? const {}, w);
      return;
    }
    switch (f.type) {
      case 'bool': w.u8((v == true || v == 1) ? 1 : 0); break;
      case 'byte': case 'uint8': case 'char': w.u8((v as num?)?.toInt() ?? 0); break;
      case 'int8': w.i8((v as num?)?.toInt() ?? 0); break;
      case 'int16': w.i16((v as num?)?.toInt() ?? 0); break;
      case 'uint16': w.u16((v as num?)?.toInt() ?? 0); break;
      case 'int32': w.i32((v as num?)?.toInt() ?? 0); break;
      case 'uint32': w.u32((v as num?)?.toInt() ?? 0); break;
      case 'int64': w.i64((v as num?)?.toInt() ?? 0); break;
      case 'uint64': w.u64((v as num?)?.toInt() ?? 0); break;
      case 'float32': w.f32((v as num?)?.toDouble() ?? 0); break;
      case 'float64': w.f64((v as num?)?.toDouble() ?? 0); break;
      case 'string': case 'wstring': w.str(v?.toString() ?? ''); break;
    }
  }
}
