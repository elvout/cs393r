#ifndef SRC_SLAM_COMMON_HH_
#define SRC_SLAM_COMMON_HH_

#include "util/profiling.h"

namespace slam {
namespace common {

util::DurationDistribution& runtime_dist();

}  // namespace common
}  // namespace slam

#endif  // SRC_SLAM_COMMON_HH_
