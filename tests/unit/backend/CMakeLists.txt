# Tests that link GTSAM. Added only when MSS_WITH_GTSAM is ON.
# Imported targets are directory-scoped: core/ found GTSAM, so find it again here.
find_package(GTSAM REQUIRED)
mss_add_test(test_lie_gtsam test_lie_gtsam.cpp)
target_link_libraries(test_lie_gtsam PRIVATE gtsam)
