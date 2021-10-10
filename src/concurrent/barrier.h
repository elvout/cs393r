#ifndef SRC_CONCURRENT_BARRIER_H_
#define SRC_CONCURRENT_BARRIER_H_

#include <condition_variable>
#include <mutex>

namespace concurrent {

/**
 * A simple reusable barrier.
 */
class Barrier {
 public:
  Barrier(unsigned int capacity);
  Barrier(const Barrier&) = delete;
  Barrier& operator=(const Barrier&) = delete;

  void wait();

 private:
  const unsigned int capacity_;
  unsigned int waiting_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

}  // namespace concurrent

#endif  // SRC_CONCURRENT_BARRIER_H_
