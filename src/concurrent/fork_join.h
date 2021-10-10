#ifndef SRC_CONCURRENT_FORK_JOIN_H_
#define SRC_CONCURRENT_FORK_JOIN_H_

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "concurrent/barrier.h"

namespace concurrent {

/**
 * A base Task class for the `ForkJoin` executor class.
 *
 * The derived class should store all arguments for the function as
 * data members.
 *
 * Synchronization is abstracted by the `ForkJoin` class.
 *
 * Example:
 * ```c++
 *  struct MyTask : ForkJoinTask {
 *    void operator()() override {
 *      std::vector<int>& data = *data_p;
 *      for (int& datum : data) {
 *        // do something with datum
 *      }
 *
 *      call_a_function();
 *    }
 *
 *    int task_id;
 *    std::vector<int>* data_p;
 *  };
 * ```
 */
struct ForkJoinTask {
  virtual void operator()() = 0;
  virtual ~ForkJoinTask() {}
};

/**
 * A reusable fork-join executor with a fixed-size thread pool.
 */
class ForkJoin {
 public:
  /**
   * Construct a ForkJoin.
   *
   * The constructor should be called with `vector<shared_ptr<DerivedTask>>`.
   */
  ForkJoin(std::vector<std::shared_ptr<ForkJoinTask>>& tasks);
  ~ForkJoin();

  void fork();
  void join();

 private:
  std::vector<std::thread> threads_;
  Barrier start_exec_;
  Barrier end_exec_;
  std::mutex fork_mutex_;
  bool run_;
};

}  // namespace concurrent

#endif  // SRC_CONCURRENT_FORK_JOIN_H_
