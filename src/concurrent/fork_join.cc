#include "concurrent/fork_join.h"

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace concurrent {

ForkJoin::ForkJoin(std::vector<std::shared_ptr<ForkJoinTask>>& tasks)
    : threads_(),
      start_exec_(tasks.size() + 1),
      end_exec_(tasks.size() + 1),
      fork_mutex_(),
      run_(true) {
  const size_t N_THREADS = tasks.size();

  threads_.reserve(N_THREADS);
  for (size_t i = 0; i < N_THREADS; i++) {
    std::shared_ptr<ForkJoinTask> task_p = tasks[i];

    /**
     * Wrap the caller's task with fork-join synchronization.
     */
    threads_.emplace_back([=] {
      while (true) {
        this->start_exec_.wait();
        if (!(this->run_)) {
          break;
        }
        (*task_p)();
        this->end_exec_.wait();
      }
    });
  }
}

ForkJoin::~ForkJoin() {
  // We must enforce mutual exclusion with `fork()`. Otherwise,
  // `fork()` could be called after `run_` is set to true, causing
  // this function to permanently block.
  std::unique_lock lock(fork_mutex_);

  run_ = false;
  // Wait until worker threads are blocked on start_exec_,
  // then force an unblock.
  start_exec_.wait();

  for (auto& thread : threads_) {
    thread.join();
  }
}

void ForkJoin::fork() {
  // Mutual exclusion with the destructor.
  std::unique_lock lock(fork_mutex_);

  if (run_) {
    start_exec_.wait();
  }
}

void ForkJoin::join() {
  end_exec_.wait();
}

}  // namespace concurrent
