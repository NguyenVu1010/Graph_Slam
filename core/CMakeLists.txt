# core: C++17 library, no ROS. Dependency direction: pipeline -> modules -> common.
cmake_minimum_required(VERSION 3.22)
project(mss_core VERSION 0.0.1 LANGUAGES CXX)

option(MSS_WITH_GTSAM "Link GTSAM into core/backend (OFF builds the backend as a stub)" ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(mss_warnings)
include(GNUInstallDirs)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
if(MSS_WITH_GTSAM)
  find_package(GTSAM REQUIRED)
endif()

set(MSS_MODULES common preprocess map scan_match degeneracy imu gnss odom loop backend pipeline)
foreach(module IN LISTS MSS_MODULES)
  add_subdirectory(${module})
endforeach()

# Export so ros2_node and external builds can find_package(mss_core).
set(MSS_TARGETS)
foreach(module IN LISTS MSS_MODULES)
  list(APPEND MSS_TARGETS mss_${module})
endforeach()
install(TARGETS ${MSS_TARGETS} EXPORT mss_coreTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
foreach(module IN LISTS MSS_MODULES)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${module}/include")
    install(DIRECTORY ${module}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  endif()
endforeach()
install(EXPORT mss_coreTargets NAMESPACE mss:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mss_core)
include(CMakePackageConfigHelpers)
configure_package_config_file(../cmake/mss_coreConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/mss_coreConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mss_core)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mss_coreConfig.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mss_core)
