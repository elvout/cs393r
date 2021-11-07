// Adapted from
// https://github.com/elvout/cxx_thread_pool/tree/aa894408ecb325a315372e53f63a3d040c87ad90

#include "thread_pool.hh"

#include <iostream>

namespace concurrent {

ThreadPool::ThreadPool(size_t n_threads)
    : threads_(),
      tasks_(),
      tasks_mutex_(),
      tasks_ready_(),
      completed_task_count_(0),
      destructor_called_(false) {
  threads_.reserve(n_threads);
  std::cout << "[ThreadPool INFO]: using " << n_threads << " threads\n";
  for (size_t i = 0; i < n_threads; i++) {
    threads_.emplace_back([=] {
      while (true) {
        std::function<void()> fn = this->get();
        if (!fn) {
          break;
        }

        fn();
        this->completed_task_count_.fetch_add(1);
      }
    });
  }
}

// Destructor cases:
//
// 1. task queue is empty, all threads waiting on condvar
//
// 2. task queue size < N_THREADS
//    Some threads are currently executing and could poll again
//    Some threads could be polling now
//
// 3. task queue size > N_THREADS
//    put() might be being called
//    threads are currently executing or polling new tasks
//
ThreadPool::~ThreadPool() {
  destructor_called_ = true;

  {
    std::unique_lock<std::mutex> lock(tasks_mutex_);
    tasks_ready_.notify_all();
  }

  std::cout << "[~ThreadPool INFO]: waiting for threads to complete" << std::endl;
  for (auto& thread : threads_) {
    thread.join();
  }

  std::cout << "[~ThreadPool INFO]: completed task count: " << completed_task_count_.load()
            << std::endl;
}

void ThreadPool::put(std::function<void()> task) {
  std::unique_lock<std::mutex> lock(tasks_mutex_);

  tasks_.push_back(std::move(task));
  tasks_ready_.notify_one();
}

std::function<void()> ThreadPool::get() {
  std::unique_lock<std::mutex> lock(tasks_mutex_);

  if (destructor_called_) {
    return std::function<void()>();
  }

  while (tasks_.empty()) {
    tasks_ready_.wait(lock);

    if (destructor_called_) {
      return std::function<void()>();
    }
  }

  std::function<void()> task_copy = tasks_.front();
  tasks_.pop_front();
  return task_copy;
}

}  // namespace concurrent
