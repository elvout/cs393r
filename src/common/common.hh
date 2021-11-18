#ifndef SRC_COMMON_COMMON_HH_
#define SRC_COMMON_COMMON_HH_

#include "concurrent/thread_pool.hh"
#include "util/profiling.h"

namespace common {

extern util::DurationDistribution runtime_dist;
extern concurrent::ThreadPool thread_pool;

}  // namespace common

#endif  // SRC_COMMON_COMMON_HH_
