#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "particle_filter.h"

using std::vector;

namespace {
constexpr size_t N_THREADS = 4;

class Barrier {
 public:
  Barrier(unsigned int count) : count(count), waiting(0), mutex(), cond() {
    if (count == 0) {
      printf("ERROR: BARRIER INITIALIZED WITH 0 COUNT\n");
      exit(1);
    }
  }
  Barrier(const Barrier&) = delete;
  Barrier& operator=(const Barrier&) = delete;

  void wait() {
    std::unique_lock<std::mutex> lock(mutex);

    waiting++;
    if (waiting == count) {
      waiting = 0;
      cond.notify_all();
      lock.unlock();
    } else {
      cond.wait(lock);
      lock.unlock();
    }
  }

 private:
  const unsigned int count;
  unsigned int waiting;
  std::mutex mutex;
  std::condition_variable cond;
};

struct WorkerArgs {
  const size_t id;

  particle_filter::ParticleFilter* pfilter;
  std::vector<particle_filter::Particle>* particles;
  size_t start_idx;
  size_t end_idx;

  std::vector<float>* sampled_ranges;
  float range_min;
  float range_max;
  float angle_min;
  float angle_max;

  WorkerArgs(size_t id) : id(id) {}
};

void worker_function(WorkerArgs* args, Barrier* start_exec, Barrier* end_exec, const bool* run_) {
  while (true) {
    start_exec->wait();

    if (!(*run_)) {
      break;
    }

    particle_filter::ParticleFilter* pfilter = args->pfilter;
    std::vector<particle_filter::Particle>& particles = *args->particles;
    size_t start_idx = args->start_idx;
    size_t end_idx = args->end_idx;
    std::vector<float>& ranges = *args->sampled_ranges;
    float range_min = args->range_min;
    float range_max = args->range_max;
    float angle_min = args->angle_min;
    float angle_max = args->angle_max;

    for (size_t idx = start_idx; idx < end_idx; idx++) {
      pfilter->Update(ranges, range_min, range_max, angle_min, angle_max, particles[idx]);
    }

    end_exec->wait();
  }
}

class ForkJoin {
 public:
  ForkJoin(size_t n_threads) : start_exec_(n_threads + 1), end_exec_(n_threads + 1), run_(true) {
    threads_.reserve(n_threads);
    worker_args_.reserve(n_threads);

    for (size_t i = 0; i < n_threads; i++) {
      worker_args_.emplace_back(i);
    }

    Barrier* start_exec_p = &start_exec_;
    Barrier* end_exec_p = &end_exec_;
    bool* run_p = &run_;
    for (size_t i = 0; i < n_threads; i++) {
      WorkerArgs* arg = &worker_args_[i];
      std::thread worker =
          std::thread([=] { worker_function(arg, start_exec_p, end_exec_p, run_p); });
      threads_.push_back(std::move(worker));
    }
  }

  ~ForkJoin() {
    std::unique_lock lock(fork_mutex_);

    run_ = false;

    // wait until worker threads are blocked on start_exec_
    // then force an unblock
    start_exec_.wait();

    for (auto& thread : threads_) {
      thread.join();
    }
  }

  void fork() {
    std::unique_lock lock(fork_mutex_);
    if (run_) {
      start_exec_.wait();
    }
  }

  void join() { end_exec_.wait(); }

  std::vector<WorkerArgs>& worker_args() { return worker_args_; }

 private:
  std::vector<std::thread> threads_;
  std::vector<WorkerArgs> worker_args_;

  Barrier start_exec_;
  Barrier end_exec_;

  std::mutex fork_mutex_;
  bool run_;
};

ForkJoin executor(N_THREADS);

}  // namespace

namespace particle_filter {

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  const float range_min,
                                  const float range_max,
                                  const float angle_min,
                                  const float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // Limit updates until the robot's pose has changed significantly
  // to reduce overconfidence in the Observation Likelihood Model.
  constexpr double kMinDistThreshold = 0.1;                // meters
  constexpr double kMinAngleDistThreshold = 0.1745329252;  // 10 degrees
  static Eigen::Vector2f prev_update_loc;
  static float prev_update_angle;

  if ((prev_odom_loc_ - prev_update_loc).norm() >= kMinDistThreshold ||
      std::abs(math_util::ReflexToConvexAngle(prev_odom_angle_ - prev_update_angle)) >=
          kMinAngleDistThreshold) {
    prev_update_loc = prev_odom_loc_;
    prev_update_angle = prev_odom_angle_;
  } else {
    return;
  }

  // Sample the sensor readings before computing point cloud estimations
  // to improve performance.
  std::vector<float> ranges_sample;
  ranges_sample.reserve(ranges.size() / 10);
  for (size_t i = 0; i < ranges.size(); i += 10) {
    ranges_sample.push_back(ranges[i]);
  }
  const float sample_angle_max =
      angle_max - (angle_max - angle_min) / ranges.size() * (ranges.size() % 10);

  size_t num_particles = particles_.size();
  std::vector<WorkerArgs>& worker_args = executor.worker_args();
  for (size_t i = 0; i < N_THREADS; i++) {
    WorkerArgs& args = worker_args[i];

    args.pfilter = this;
    args.particles = &particles_;
    args.start_idx = num_particles * i / N_THREADS;
    args.end_idx = num_particles * (i + 1) / N_THREADS;
    args.sampled_ranges = &ranges_sample;
    args.range_min = range_min;
    args.range_max = range_max;
    args.angle_min = angle_min;
    args.angle_max = sample_angle_max;
  }
  executor.fork();
  executor.join();

  // Resample every n updates
  static int num_of_updates_since_last_resample = 0;
  num_of_updates_since_last_resample++;
  if (num_of_updates_since_last_resample >= 5) {
    Resample();
    num_of_updates_since_last_resample = 0;
  }
}

}  // namespace particle_filter
