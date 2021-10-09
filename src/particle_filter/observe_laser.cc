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

std::vector<WorkerArgs> worker_args;
std::vector<std::thread> threads;
Barrier work_ready(N_THREADS + 1);
Barrier comp_done(N_THREADS + 1);

void worker_function(WorkerArgs* args) {
  while (true) {
    work_ready.wait();

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

    comp_done.wait();
  }
}

struct InitObj {
  InitObj() {
    threads.reserve(N_THREADS);
    worker_args.reserve(N_THREADS);

    for (size_t i = 0; i < N_THREADS; i++) {
      worker_args.emplace_back(i);
    }

    for (size_t i = 0; i < N_THREADS; i++) {
      WorkerArgs* arg = &worker_args[i];
      threads.emplace_back([arg] { worker_function(arg); });
    }
  }
};

static InitObj _init;
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
  work_ready.wait();
  comp_done.wait();

  // Resample every n updates
  static int num_of_updates_since_last_resample = 0;
  num_of_updates_since_last_resample++;
  if (num_of_updates_since_last_resample >= 5) {
    Resample();
    num_of_updates_since_last_resample = 0;
  }
}

}  // namespace particle_filter
