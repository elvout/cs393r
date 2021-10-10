#ifndef SRC_UTIL_PROFILING_H_
#define SRC_UTIL_PROFILING_H_

#include <chrono>
#include <deque>
#include <functional>
#include <string>
#include <unordered_map>

namespace util {

class DurationDistribution {
 private:
  class DelayUntilEndOfScope {
   public:
    DelayUntilEndOfScope(std::function<void()> fn);
    ~DelayUntilEndOfScope();

   private:
    std::function<void()> fn_;
  };

  using steady_clock = std::chrono::steady_clock;
  using Duration = std::chrono::steady_clock::duration;
  using TimePoint = std::chrono::steady_clock::time_point;

 public:
  DurationDistribution(size_t history_size = 10000);

  void start_lap(std::string key);
  void end_lap(std::string key);
  DelayUntilEndOfScope auto_lap(std::string key);
  std::string summary();

 private:
  void add(std::string key, Duration val);

  const size_t history_size_;
  std::unordered_map<std::string, std::deque<Duration>> data_;
  std::unordered_map<std::string, TimePoint> last_start_tp_;
};

}  // namespace util

#endif  // SRC_UTIL_PROFILING_H_
