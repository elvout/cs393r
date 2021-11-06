// Adapted from
// https://github.com/elvout/cxx_thread_pool/tree/aa894408ecb325a315372e53f63a3d040c87ad90

#ifndef SRC_CONCURRENT_THREAD_POOL_HH_
#define SRC_CONCURRENT_THREAD_POOL_HH_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace concurrent {

class ThreadPool {
 public:
  ThreadPool(size_t n_threads);
  ~ThreadPool();

  void put(std::function<void()> task);

 private:
  std::function<void()> get();

 private:
  std::vector<std::thread> threads_;
  std::deque<std::function<void()>> tasks_;
  std::mutex tasks_mutex_;
  std::condition_variable tasks_ready_;
  std::atomic_size_t completed_task_count_;

  bool destructor_called_;
};

}  // namespace concurrent

#endif  // SRC_CONCURRENT_THREAD_POOL_HH_
