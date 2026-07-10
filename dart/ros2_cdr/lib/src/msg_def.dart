// msg_def.dart — parses ROS 2 `.msg` definitions (the "ros2msg" concatenated
// format that foxglove_bridge / rosidl ship) into field lists, so CDR de/encode
// is fully schema-driven with NO generated code.
//
// Concatenated format: the root message, then each dependency separated by a
// line of '=' preceded by "MSG: pkg/Type":
//
//   Header header
//   float64 x
//   ================================================================
//   MSG: std_msgs/Header
//   builtin_interfaces/Time stamp
//   string frame_id

const _primitives = {
  'bool', 'byte', 'char', 'int8', 'uint8', 'int16', 'uint16', 'int32',
  'uint32', 'int64', 'uint64', 'float32', 'float64', 'string', 'wstring',
};

bool isPrimitive(String t) => _primitives.contains(t);

/// One field of a message.
class Field {
  Field({
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
  final int? fixedLen; // null + isArray => length-prefixed sequence
}

/// A parsed message definition (ordered fields).
class MsgDef {
  MsgDef(this.name, this.fields);
  final String name;
  final List<Field> fields;
}

/// Normalise a raw ROS type reference to canonical "pkg/Type".
/// [parentPkg] resolves bare same-package names.
String canonicalType(String raw, [String? parentPkg]) {
  var t = raw.trim();
  final br = t.indexOf('[');
  if (br >= 0) t = t.substring(0, br);
  final lt = t.indexOf('<=');
  if (lt >= 0) t = t.substring(0, lt);
  if (t == 'Header') return 'std_msgs/Header';
  final slashes = '/'.allMatches(t).length;
  if (slashes == 2) {
    final p = t.split('/');
    return '${p[0]}/${p[2]}'; // pkg/msg/Type -> pkg/Type
  }
  if (slashes == 1) return t;
  return parentPkg == null ? t : '$parentPkg/$t';
}

/// A registry of message definitions keyed by canonical "pkg/Type".
class MsgRegistry {
  final Map<String, MsgDef> _defs = {};

  MsgDef? operator [](String canonical) => _defs[canonical];
  bool contains(String canonical) => _defs.containsKey(canonical);

  /// Parses a concatenated ros2msg schema (as sent by foxglove_bridge / stored
  /// in a .msg-with-deps blob). [rootType] is the schemaName (`pkg/msg/Type`
  /// or `pkg/Type`); its block is the FIRST one (no MSG header).
  void addConcatenated(String rootType, String schema) {
    final root = canonicalType(rootType);
    final sep = RegExp(r'^=+\s*$', multiLine: true);
    for (final block in schema.split(sep)) {
      String? name;
      final body = <String>[];
      for (final raw in block.split('\n')) {
        final line = raw.trim();
        if (line.startsWith('MSG:')) {
          name = canonicalType(line.substring(4).trim());
        } else {
          body.add(raw);
        }
      }
      final key = name ?? root;
      _defs[key] = _parseBody(key, body);
    }
  }

  /// Register a single message body (no dependencies) under [canonical].
  void addSingle(String canonical, String body) {
    final key = canonicalType(canonical);
    _defs[key] = _parseBody(key, body.split('\n'));
  }

  MsgDef _parseBody(String owner, List<String> lines) {
    final fields = <Field>[];
    final pkg = owner.contains('/') ? owner.split('/').first : owner;
    for (var raw in lines) {
      final hash = raw.indexOf('#');
      if (hash >= 0) raw = raw.substring(0, hash);
      final line = raw.trim();
      if (line.isEmpty) continue;
      final parts = line.split(RegExp(r'\s+'));
      if (parts.length < 2) continue;
      var type = parts[0];
      final rest = parts[1];
      // Constants "TYPE NAME = value" and defaults are not serialized.
      if (parts.length >= 3 && parts[2] == '=') continue;
      if (rest.contains('=')) continue;

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
      final prim = isPrimitive(type.contains('<=') ? type.split('<=').first : type);
      fields.add(Field(
        name: rest,
        type: prim ? type.split('<=').first : canonicalType(type, pkg),
        isPrimitive: prim,
        isArray: isArray,
        fixedLen: fixedLen,
      ));
    }
    return MsgDef(owner, fields);
  }
}
