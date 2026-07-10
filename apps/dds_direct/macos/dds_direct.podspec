#
# dds_direct macOS FFI plugin. Builds CycloneDDS + the libdds_direct shim from
# ../src via CMake (no cargo, no ROS), then vendors the dylibs into the app.
# Mirrors the zenoh_ffi podspec pattern.
#
Pod::Spec.new do |s|
  s.name             = 'dds_direct'
  s.version          = '0.1.0'
  s.summary          = 'Bridgeless ROS 2 over CycloneDDS-FFI + pure-Dart CDR.'
  s.description      = 'Talk to a ROS 2 graph over DDS with no ROS install; message serialization is pure Dart.'
  s.homepage         = 'https://github.com/harunkurtdev/rcldart'
  s.license          = { :file => '../LICENSE' }
  s.author           = { 'rcldart' => 'harunkurt.dev@gmail.com' }

  s.source           = { :path => '.' }
  s.source_files     = 'Classes/**/*'
  s.dependency 'FlutterMacOS'
  s.platform = :osx, '10.13'
  s.swift_version = '5.0'

  s.pod_target_xcconfig = {
    'DEFINES_MODULE' => 'YES',
    'HEADER_SEARCH_PATHS' => '$(inherited) "${PODS_TARGET_SRCROOT}/../src/build_macos/_deps/cyclonedds-src/src/core/ddsc/include" "${PODS_TARGET_SRCROOT}/../src/build_macos/_deps/cyclonedds-build/src/core/include"',
    'LIBRARY_SEARCH_PATHS' => '$(inherited) "${PODS_TARGET_SRCROOT}/../src/build_macos"',
    'OTHER_LDFLAGS' => '$(inherited) -ldds_direct -lddsc'
  }

  s.prepare_command = <<-CMD
    set -e
    export PATH="/usr/local/bin:/opt/homebrew/bin:$PATH"
    SRC_DIR="$(cd .. && pwd)/src"
    mkdir -p "${SRC_DIR}/build_macos" && cd "${SRC_DIR}/build_macos"
    if [ -f "libdds_direct.dylib" ] && [ -f "libddsc.dylib" ]; then exit 0; fi
    rm -rf CMakeCache.txt CMakeFiles
    export SDKROOT=$(xcrun --sdk macosx --show-sdk-path)
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES="$(uname -m)" -DCMAKE_OSX_SYSROOT=$SDKROOT -Wno-dev > build.log 2>&1
    cmake --build . --config Release >> build.log 2>&1 || { tail -30 build.log; exit 1; }
  CMD

  s.vendored_libraries = [
    '../src/build_macos/libdds_direct.dylib',
    '../src/build_macos/libddsc.dylib'
  ]
  s.preserve_paths = ['../src/build_macos/**/*']
  s.public_header_files = 'Classes/**/*.h'
end
