#include "concurrent/fork_join.h"

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace concurrent {

ForkJoin::ForkJoin(std::vector<std::shared_ptr<ForkJoinTask>>& tasks, std::string name)
    : threads_(),
      start_exec_(tasks.size() + 1),
      end_exec_(tasks.size() + 1),
      fork_mutex_(),
      run_(true) {
  constexpr size_t kLinuxMaxThreadNameLen = 15;
  if (name.length() >= kLinuxMaxThreadNameLen - 6) {
    name.resize(kLinuxMaxThreadNameLen - 6);
  }
  name += "worker";

  threads_.reserve(tasks.size());
  for (size_t i = 0; i < tasks.size(); i++) {
    std::shared_ptr<ForkJoinTask> task_p = tasks[i];

    /**
     * Wrap the caller's task with fork-join synchronization.
     */
    std::thread thread([=] {
      while (true) {
        this->start_exec_.wait();
        if (!(this->run_)) {
          break;
        }
        (*task_p)();
        this->end_exec_.wait();
      }
    });

    pthread_setname_np(thread.native_handle(), name.c_str());
    threads_.push_back(std::move(thread));
  }
}

ForkJoin::~ForkJoin() {
  // We must enforce mutual exclusion with `fork()`. Otherwise
  // `fork()` could be called immediately after `run_` is set to true,
  // causing this function to permanently block.
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
