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

  std::vector<particle_filter::Observation>* sampled_ranges;
  float range_min;
  float range_max;

 private:
  void exec() {
    for (size_t idx = start_idx; idx < end_idx; idx++) {
      pfilter->Update(*sampled_ranges, range_min, range_max, (*particles)[idx]);
    }

    promise_.set_value();
  }

  std::promise<void> promise_;
  std::future<void> done_;
};

std::vector<ObserveLaserTask> tasks(kObserveLaserTaskCount);

}  // namespace

namespace particle_filter {

void ParticleFilter::ObserveLaser(const sensor_msgs::LaserScan& msg) {
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
  std::vector<Observation> range_cloud(msg.ranges.size());
  for (size_t i = 0; i < msg.ranges.size(); i++) {
    range_cloud[i].range = msg.ranges[i];
    range_cloud[i].msg_idx = i;
    range_cloud[i].valid = (msg.ranges[i] > msg.range_min && msg.ranges[i] < msg.range_max);
    range_cloud[i].angle = msg.angle_min + i * msg.angle_increment;
  }

  std::vector<Observation> sampled_cloud = DensitySampledPointCloud(range_cloud);

  size_t num_particles = particles_.size();
  for (size_t i = 0; i < tasks.size(); i++) {
    ObserveLaserTask& task = tasks[i];

    task.pfilter = this;
    task.particles = &particles_;
    task.start_idx = num_particles * i / tasks.size();
    task.end_idx = num_particles * (i + 1) / tasks.size();
    task.sampled_ranges = &sampled_cloud;
    task.range_min = msg.range_min;
    task.range_max = msg.range_max;
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
