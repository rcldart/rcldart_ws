# Shared native build for the dds_direct Flutter FFI plugin, invoked by each
# platform (linux/CMakeLists.txt, android/build.gradle externalNativeBuild,
# windows/CMakeLists.txt; macOS/iOS build the same C via CocoaPods). Fetches
# CycloneDDS (the ONLY native dep — no ROS) and builds libdds_direct.
cmake_minimum_required(VERSION 3.10)
project(dds_direct LANGUAGES C)

include(FetchContent)

# Build CycloneDDS as a STATIC lib and link it INTO libdds_direct.so, so the
# plugin ships exactly ONE native library (no separate libddsc + no versioned
# SONAME to reconcile when Flutter bundles it). PIC so it links into a .so.
set(BUILD_SHARED_LIBS       OFF CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE BOOL "" FORCE)
set(ENABLE_SSL              OFF CACHE BOOL "" FORCE)
set(ENABLE_SECURITY         OFF CACHE BOOL "" FORCE)
set(ENABLE_SHM              OFF CACHE BOOL "" FORCE)
set(ENABLE_TYPE_DISCOVERY   ON  CACHE BOOL "" FORCE)
set(ENABLE_TOPIC_DISCOVERY  ON  CACHE BOOL "" FORCE)
set(BUILD_IDLC              OFF CACHE BOOL "" FORCE)
set(BUILD_IDLC_TESTING      OFF CACHE BOOL "" FORCE)
set(BUILD_DDSPERF           OFF CACHE BOOL "" FORCE) # tool needs idlc; we don't ship it
set(BUILD_EXAMPLES         OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING          OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  cyclonedds
  GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
  GIT_TAG        0.10.5
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(cyclonedds)

# The FFI shim. `dds_direct` is the name the Dart side dlopen()s (libdds_direct.so).
add_library(dds_direct SHARED "${CMAKE_CURRENT_SOURCE_DIR}/ddsros.c")
set_target_properties(dds_direct PROPERTIES
  PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/ddsros.h"
  C_STANDARD 11
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME "dds_direct")
# Link CycloneDDS statically into the shim (whole-archive so the ddsi_sertype_*
# internals we reference are kept even though nothing else pulls them).
target_link_libraries(dds_direct PRIVATE CycloneDDS::ddsc)

# The sertype/serdata API lives in CycloneDDS "internal" headers (dds/ddsi/*,
# dds/ddsrt/*) that the CycloneDDS::ddsc target does NOT export — rmw_cyclonedds
# uses them the same way. Add the source + generated include dirs explicitly.
target_include_directories(dds_direct PRIVATE
  "${cyclonedds_SOURCE_DIR}/src/core/ddsi/include"
  "${cyclonedds_SOURCE_DIR}/src/core/ddsc/include"
  "${cyclonedds_SOURCE_DIR}/src/ddsrt/include"
  "${cyclonedds_BINARY_DIR}/src/ddsrt/include"     # generated: features.h, version.h
  "${cyclonedds_BINARY_DIR}/src/core/include"      # generated: dds/export.h
)

# Exactly one native lib ships: libdds_direct.so (CycloneDDS is baked in).
set(dds_direct_bundled_libraries "" PARENT_SCOPE)
