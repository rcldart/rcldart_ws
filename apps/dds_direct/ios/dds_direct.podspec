#
# dds_direct iOS FFI plugin. Builds CycloneDDS + libdds_direct from ../src via
# CMake with the iOS toolchain, then vendors the static/dynamic libs.
# NOTE: iOS cross-compile of CycloneDDS needs the iOS CMake toolchain flags below
# and a device/simulator arch split — verify on a Mac (mirrors zenoh_ffi/ios).
#
Pod::Spec.new do |s|
  s.name             = 'dds_direct'
  s.version          = '0.1.0'
  s.summary          = 'Bridgeless ROS 2 over CycloneDDS-FFI + pure-Dart CDR.'
  s.description      = 'Talk to a ROS 2 graph over DDS with no ROS install; serialization is pure Dart.'
  s.homepage         = 'https://github.com/harunkurtdev/rcldart'
  s.license          = { :file => '../LICENSE' }
  s.author           = { 'rcldart' => 'harunkurt.dev@gmail.com' }

  s.source           = { :path => '.' }
  s.source_files     = 'Classes/**/*'
  s.dependency 'Flutter'
  s.platform = :ios, '13.0'
  s.swift_version = '5.0'

  s.pod_target_xcconfig = {
    'DEFINES_MODULE' => 'YES',
    'EXCLUDED_ARCHS[sdk=iphonesimulator*]' => 'i386',
    'HEADER_SEARCH_PATHS' => '$(inherited) "${PODS_TARGET_SRCROOT}/../src/build_ios/_deps/cyclonedds-src/src/core/ddsc/include" "${PODS_TARGET_SRCROOT}/../src/build_ios/_deps/cyclonedds-build/src/core/include"',
    'LIBRARY_SEARCH_PATHS' => '$(inherited) "${PODS_TARGET_SRCROOT}/../src/build_ios"',
    'OTHER_LDFLAGS' => '$(inherited) -ldds_direct -lddsc'
  }

  s.prepare_command = <<-CMD
    set -e
    export PATH="/usr/local/bin:/opt/homebrew/bin:$PATH"
    SRC_DIR="$(cd .. && pwd)/src"
    mkdir -p "${SRC_DIR}/build_ios" && cd "${SRC_DIR}/build_ios"
    if [ -f "libdds_direct.a" ] || [ -f "libdds_direct.dylib" ]; then exit 0; fi
    rm -rf CMakeCache.txt CMakeFiles
    cmake .. -G Xcode \
      -DCMAKE_SYSTEM_NAME=iOS \
      -DCMAKE_OSX_DEPLOYMENT_TARGET=13.0 \
      -DCMAKE_OSX_ARCHITECTURES=arm64 \
      -DBUILD_SHARED_LIBS=OFF -Wno-dev > build.log 2>&1
    cmake --build . --config Release >> build.log 2>&1 || { tail -30 build.log; exit 1; }
  CMD

  s.vendored_libraries = ['../src/build_ios/libdds_direct.a', '../src/build_ios/libddsc.a']
  s.preserve_paths = ['../src/build_ios/**/*']
  s.public_header_files = 'Classes/**/*.h'
end
