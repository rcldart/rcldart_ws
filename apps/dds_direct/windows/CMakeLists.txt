cmake_minimum_required(VERSION 3.14)
set(PROJECT_NAME "dds_direct")
project(${PROJECT_NAME} LANGUAGES C)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../src" "${CMAKE_CURRENT_BINARY_DIR}/shared")

set(dds_direct_bundled_libraries
  $<TARGET_FILE:dds_direct>
  PARENT_SCOPE
)
