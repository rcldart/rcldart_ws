# ros2_cdr

Pure-Dart **ROS 2 CDR codec** + `.msg` schema parser. Encodes/decodes ROS 2
messages to/from plain Dart maps over CDR (XCDR1) — with **no native code, no
rosidl typesupport, and no ROS install**. It is the serialization core shared by
[`dds_direct`](../../apps/dds_direct), [`zenoh_ros2`](../../apps/zenoh_ros2) and
the `flutglove_*` apps.

Try it live: the [rcldart site](https://rcldart.github.io/rcldart/) runs this exact
encoder in the browser.

## Why

ROS 2 puts CDR bytes on the wire. Normally decoding them needs the per-type
`rosidl` C typesupport (and therefore a ROS install + codegen). `ros2_cdr` reads
the message **schema** (the `.msg` text the graph already carries) and does the
CDR by hand in Dart — so any type de/serializes with nothing installed.

## Use

```dart
import 'package:ros2_cdr/ros2_cdr.dart';

final reg = MsgRegistry()
  ..addConcatenated('std_msgs/msg/String', 'string data');
final codec = Ros2Codec(reg);

final bytes = codec.encode('std_msgs/msg/String', {'data': 'hello'}); // -> CDR
final msg   = codec.decode('std_msgs/msg/String', bytes);             // -> {data: hello}
```

For nested types, pass the concatenated `ros2msg` schema (root body, then each
dependency after a `MSG: pkg/Type` header) — exactly what `foxglove_bridge` sends
and what `dds_direct` bundles.

## What's inside

| File | Role |
|------|------|
| `src/cdr.dart` | `CdrReader` / `CdrWriter` — aligned XCDR1, encapsulation header, LE/BE |
| `src/msg_def.dart` | `MsgRegistry` — parse concatenated `.msg` schemas → fields |
| `src/codec.dart` | `Ros2Codec` — recursive map ⇄ CDR (nested, arrays, sequences, strings) |

Handles primitives, bounded/unbounded sequences, fixed arrays, byte arrays
(`uint8[]` → `Uint8List`), nested messages, and both endiannesses. Unit-tested
(`dart test`).
