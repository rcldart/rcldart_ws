// shared rosidl_runtime_c helper types (generated once per package)
import 'dart:ffi' as ffi;

final class rosidl_runtime_c__String extends ffi.Struct {
  external ffi.Pointer<ffi.Char> data;
  @ffi.Size()
  external int size;
  @ffi.Size()
  external int capacity;
}

final class rosidl_runtime_c__String__Sequence extends ffi.Struct {
  external ffi.Pointer<rosidl_runtime_c__String> data;
  @ffi.Size()
  external int size;
  @ffi.Size()
  external int capacity;
}

final class rosidl_runtime_c__uint8__Sequence extends ffi.Struct {
  external ffi.Pointer<ffi.Uint8> data;
  @ffi.Size()
  external int size;
  @ffi.Size()
  external int capacity;
}

