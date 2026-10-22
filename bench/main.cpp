// Latency benchmark. Runs on Pi5 without ROS. Reports p50 / p95 / CPU per stage.
// Usage (once the pipeline exists): mss_bench <config.yaml> <dataset-dir>
#include <cstdio>

#include "mss/common/logger.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <config.yaml> <dataset-dir>\n", argv[0]);
    return 2;
  }
  mss::common::log_info("mss_bench: pipeline not implemented yet");
  return 0;
}
