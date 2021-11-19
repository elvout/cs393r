#include <cstdio>
#include <functional>
#include <future>
#include <memory>
#include <vector>
#include "common/common.hh"
#include "eigen3/Eigen/Dense"
#include "particle_filter.h"

using std::vector;

namespace {
constexpr size_t kObserveLaserTaskCount = 4;

class ObserveLaserTask {
 public:
  std::function<void()> as_fn() {
    promise_ = std::promise<void>();
    done_ = promise_.get_future();

    return std::bind(&ObserveLaserTask::exec, this);
  }

  void wait_for_completion() { done_.wait(); }

  particle_filter::ParticleFilter* pfilter;
  std::vector<particle_filter::Particle>* particles;
  size_t start_idx;
  size_t end_idx;

  // std::vector<float>* sampled_ranges;
  std::vector<particle_filter::Observation>* sampled_ranges;
  size_t full_size;
  float range_min;
  float range_max;
  float angle_min;
  float angle_max;

 private:
  void exec() {
    for (size_t idx = start_idx; idx < end_idx; idx++) {
      pfilter->Update(*sampled_ranges, full_size, range_min, range_max, angle_min, angle_max,
                      (*particles)[idx]);
    }

    promise_.set_value();
  }

  std::promise<void> promise_;
  std::future<void> done_;
};

std::vector<ObserveLaserTask> tasks(kObserveLaserTaskCount);

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
  // std::vector<float> ranges_sample;
  // ranges_sample.reserve(ranges.size() / 10);
  // for (size_t i = 0; i < ranges.size(); i += 10) {
  //   ranges_sample.push_back(ranges[i]);
  // }
  // const float sample_angle_max =
  //     angle_max - (angle_max - angle_min) / ranges.size() * (ranges.size() % 10);
  std::vector<Observation> range_cloud(ranges.size());
  for (int i = 0; i < ranges.size(); i++) {
    range_cloud[i].range = ranges[i];
    range_cloud[i].msg_idx = i;
    range_cloud[i].valid = (ranges[i] > range_min && ranges[i] < range_max);
  }

  std::vector<Observation> sampled_cloud = DensitySampledPointCloud(range_cloud);

  size_t num_particles = particles_.size();
  for (size_t i = 0; i < tasks.size(); i++) {
    ObserveLaserTask& task = tasks[i];

    task.pfilter = this;
    task.particles = &particles_;
    task.start_idx = num_particles * i / tasks.size();
    task.end_idx = num_particles * (i + 1) / tasks.size();
    // task.sampled_ranges = &ranges_sample;
    task.sampled_ranges = &sampled_cloud;
    task.full_size = ranges.size();
    task.range_min = range_min;
    task.range_max = range_max;
    task.angle_min = angle_min;
    // task.angle_max = sample_angle_max;
    task.angle_max = angle_max;
    common::thread_pool.put(task.as_fn());
  }

  for (ObserveLaserTask& task : tasks) {
    task.wait_for_completion();
  }

  // Resample every n updates
  static int num_of_updates_since_last_resample = 0;
  num_of_updates_since_last_resample++;
  if (num_of_updates_since_last_resample >= 5) {
    Resample();
    num_of_updates_since_last_resample = 0;
  }
}

}  // namespace particle_filter
