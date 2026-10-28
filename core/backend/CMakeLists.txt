# backend: the ONLY module that includes GTSAM. Other modules return core types.
# GTSAM is linked PRIVATE: no GTSAM type or define appears in the public headers (pimpl).
# MSS_WITH_GTSAM=OFF builds src/graph_stub.cpp instead of the GTSAM sources: the same API;
# optimize() and keyframe_state() return Result::error("built without GTSAM"), other calls
# are no-ops (see graph.hpp). Derivations: docs/derivations/backend_factors.md; ADR-015.
if(MSS_WITH_GTSAM)
  add_library(mss_backend STATIC
    src/graph.cpp src/conversions.cpp src/validate.cpp
    src/imu_factor.cpp src/gnss_factor.cpp src/odom_factor.cpp src/pose_prior_factor.cpp
    src/relative_pose_factor.cpp src/static_gravity_factor.cpp)
  target_link_libraries(mss_backend PRIVATE gtsam mss::odom)
  # GTSAM already links TBB; the explicit link is for <tbb/global_control.h> in graph.cpp
  # (ADR-018). PRIVATE, but a STATIC library exports it as $<LINK_ONLY:TBB::tbb>, so
  # cmake/mss_coreConfig.cmake.in finds TBB for consumers. GTSAM_USE_TBB comes from
  # GTSAM's CMake config (gtsam_extra.cmake).
  if(GTSAM_USE_TBB)
    find_package(TBB REQUIRED)
    target_link_libraries(mss_backend PRIVATE TBB::tbb)
  endif()
else()
  add_library(mss_backend STATIC src/graph_stub.cpp src/validate.cpp)
endif()
add_library(mss::backend ALIAS mss_backend)
mss_set_target_defaults(mss_backend)
target_include_directories(mss_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mss_backend PUBLIC mss::common mss::imu)  # ImuConfig in Graph ctor
