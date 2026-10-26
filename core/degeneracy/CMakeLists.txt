# degeneracy: eigen-analysis of the scan-matching Hessian, projected update and covariance
# inflation (docs/derivations/degeneracy.md). Fixed-size Eigen only.
add_library(mss_degeneracy STATIC src/degeneracy.cpp)
add_library(mss::degeneracy ALIAS mss_degeneracy)
mss_set_target_defaults(mss_degeneracy)
target_include_directories(mss_degeneracy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_degeneracy PUBLIC mss::common)
