# io: mssdata dataset reader/writer (ADR-014, docs/mssdata_format.md) and the PCD map writer.
# App-level module used by bench and tests; pipeline does not link it. yaml-cpp parses
# index.json (JSON is YAML).
add_library(mss_io STATIC src/dataset_reader.cpp src/dataset_writer.cpp src/pcd_writer.cpp)
add_library(mss::io ALIAS mss_io)
mss_set_target_defaults(mss_io)
target_include_directories(mss_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_io PUBLIC mss::common mss::gnss)
target_link_libraries(mss_io PRIVATE yaml-cpp::yaml-cpp)
