#ifndef SRC_SLAM_COMMON_HH_
#define SRC_SLAM_COMMON_HH_

#include <functional>
#include "util/profiling.h"

namespace slam {
namespace common {

util::DurationDistribution& runtime_dist();
void thread_pool_exec(std::function<void()> task);

}  // namespace common
}  // namespace slam

#endif  // SRC_SLAM_COMMON_HH_
