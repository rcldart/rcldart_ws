# Flutter Linux plugin entry — delegates to the shared src/ build and tells the
# tooling which libraries to bundle into the app.
cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME "dds_direct")
# C AND CXX: the Flutter runner project is CXX-only, but CycloneDDS's configure
# checks (check_include_file) need C enabled/cached at this project scope.
project(${PROJECT_NAME} LANGUAGES C CXX)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../src" "${CMAKE_CURRENT_BINARY_DIR}/shared")

# One lib to bundle — CycloneDDS is statically linked into it.
set(dds_direct_bundled_libraries
  $<TARGET_FILE:dds_direct>
  PARENT_SCOPE
)
