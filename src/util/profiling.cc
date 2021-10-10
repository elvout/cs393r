#include "util/profiling.h"

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace util {

DurationDistribution::DelayUntilEndOfScope::DelayUntilEndOfScope(std::function<void()> fn)
    : fn_(fn) {}
DurationDistribution::DelayUntilEndOfScope::~DelayUntilEndOfScope() {
  fn_();
}

DurationDistribution::DurationDistribution(size_t history_size)
    : history_size_(history_size), data_(), last_start_tp_() {}

void DurationDistribution::start_lap(std::string key) {
  last_start_tp_[key] = steady_clock::now();
}

void DurationDistribution::end_lap(std::string key) {
  TimePoint end = steady_clock::now();

  auto start_it = last_start_tp_.find(key);
  if (start_it != last_start_tp_.end()) {
    TimePoint start = start_it->second;
    last_start_tp_.erase(start_it);

    add(key, end - start);
  }
}

DurationDistribution::DelayUntilEndOfScope DurationDistribution::auto_lap(std::string key) {
  start_lap(key);
  return DelayUntilEndOfScope(std::function<void()>([=] { this->end_lap(key); }));
}

std::string DurationDistribution::summary() {
  using milliseconds = std::chrono::duration<double, std::milli>;
  static const std::vector<double> percentiles = {50, 75, 90, 99, 99.9, 99.99};

  std::ostringstream output;
  output << "==== Latency Distribution ====\n";
  output << std::fixed << std::setprecision(3);

  for (auto& [key, history] : data_) {
    output << "\t" << key << "\n";

    std::sort(history.begin(), history.end());
    size_t n = history.size();

    output << "\t\tn: " << n << "\n";
    for (double percentile : percentiles) {
      auto& point = history[n * percentile / 100];
      double point_ms = milliseconds(point).count();
      output << "\t\t" << percentile << "th percentile: " << point_ms << " ms \n";
    }

    output << "\t\tmax: " << milliseconds(history.back()).count() << " ms\n";
  }

  return output.str();
}

/**
 * Add a Duration with history size checking.
 */
void DurationDistribution::add(std::string key, Duration val) {
  std::deque<Duration>& history = data_[key];

  if (history.size() == history_size_) {
    history.pop_front();
  }
  history.push_back(val);
}

}  // namespace util
