#include "common/common.hh"

#include <thread>
#include "concurrent/thread_pool.hh"
#include "util/profiling.h"

namespace common {

util::DurationDistribution runtime_dist;
concurrent::ThreadPool thread_pool(std::thread::hardware_concurrency());

}  // namespace common
