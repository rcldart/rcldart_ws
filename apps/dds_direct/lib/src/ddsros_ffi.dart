// ddsros_ffi.dart — Dart FFI bindings to the native `dds_direct` plugin lib.
//
// As a Flutter FFI plugin, the native library is built + bundled per platform by
// the tooling (see linux/android/macos/ios/windows); we open it by the standard
// plugin convention. Outside Flutter (pure `dart run`) point RCLDART_DDS_LIB at a
// built libdds_direct.so.
import 'dart:ffi';
import 'dart:io';

import 'package:ffi/ffi.dart';

const String _libName = 'dds_direct';

DynamicLibrary _open() {
  final override = Platform.environment['RCLDART_DDS_LIB'];
  if (override != null && override.isNotEmpty) return DynamicLibrary.open(override);
  if (Platform.isIOS) return DynamicLibrary.process();
  if (Platform.isMacOS) return DynamicLibrary.open('$_libName.framework/$_libName');
  if (Platform.isAndroid || Platform.isLinux) return DynamicLibrary.open('lib$_libName.so');
  if (Platform.isWindows) return DynamicLibrary.open('$_libName.dll');
  throw UnsupportedError('unsupported platform: ${Platform.operatingSystem}');
}

typedef _ParticipantC = Int32 Function(Uint32);
typedef _ParticipantD = int Function(int);
typedef _EndpointC = Int32 Function(Int32, Pointer<Utf8>, Pointer<Utf8>);
typedef _EndpointD = int Function(int, Pointer<Utf8>, Pointer<Utf8>);
typedef _WriteC = Int32 Function(Int32, Pointer<Uint8>, Size);
typedef _WriteD = int Function(int, Pointer<Uint8>, int);
typedef _TakeC = Int32 Function(Int32, Pointer<Uint8>, Size);
typedef _TakeD = int Function(int, Pointer<Uint8>, int);
typedef _WaitC = Int32 Function(Int32, Int32);
typedef _WaitD = int Function(int, int);
typedef _DeleteC = Void Function(Int32);
typedef _DeleteD = void Function(int);
typedef _DiscoReaderC = Int32 Function(Int32);
typedef _DiscoReaderD = int Function(int);
typedef _DiscoverC = Int32 Function(Int32, Pointer<Uint8>, Size);
typedef _DiscoverD = int Function(int, Pointer<Uint8>, int);

/// Thin binding object over the shim. Construct once.
class DdsRosNative {
  DdsRosNative() : _lib = _open();
  final DynamicLibrary _lib;

  late final participant = _lib.lookupFunction<_ParticipantC, _ParticipantD>('ddsros_participant');
  late final writer = _lib.lookupFunction<_EndpointC, _EndpointD>('ddsros_writer');
  late final reader = _lib.lookupFunction<_EndpointC, _EndpointD>('ddsros_reader');
  late final write = _lib.lookupFunction<_WriteC, _WriteD>('ddsros_write');
  late final take = _lib.lookupFunction<_TakeC, _TakeD>('ddsros_take');
  late final wait = _lib.lookupFunction<_WaitC, _WaitD>('ddsros_wait');
  late final del = _lib.lookupFunction<_DeleteC, _DeleteD>('ddsros_delete');
  late final discoReader = _lib.lookupFunction<_DiscoReaderC, _DiscoReaderD>('ddsros_disco_reader');
  late final discover = _lib.lookupFunction<_DiscoverC, _DiscoverD>('ddsros_discover');
}
