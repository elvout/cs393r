#include "common.hh"

#include "util/profiling.h"

namespace {
util::DurationDistribution runtime_dist_;
}

namespace slam {
namespace common {
util::DurationDistribution& runtime_dist() {
  return runtime_dist_;
}
}  // namespace common
}  // namespace slam
