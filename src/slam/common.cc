#include "common.hh"

#include <functional>
#include <thread>
#include "concurrent/thread_pool.hh"
#include "util/profiling.h"

namespace {
util::DurationDistribution runtime_dist_;
concurrent::ThreadPool slam_thread_pool_(std::thread::hardware_concurrency());
}  // namespace

namespace slam {
namespace common {

util::DurationDistribution& runtime_dist() {
  return runtime_dist_;
}

void thread_pool_exec(std::function<void()> task) {
  slam_thread_pool_.put(std::move(task));
}

}  // namespace common
}  // namespace slam
