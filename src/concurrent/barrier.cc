#include "concurrent/barrier.h"

#include <condition_variable>
#include <mutex>
#include <stdexcept>

namespace concurrent {

Barrier::Barrier(unsigned int capacity) : capacity_(capacity), waiting_(0), mutex_(), cond_() {
  if (capacity_ == 0) {
    throw std::invalid_argument("Barrier: capacity cannot be zero");
  }
}

void Barrier::wait() {
  std::unique_lock lock(mutex_);

  waiting_++;
  if (waiting_ == capacity_) {
    waiting_ = 0;
    cond_.notify_all();
    lock.unlock();
  } else {
    cond_.wait(lock);
    lock.unlock();
  }
}

}  // namespace concurrent
