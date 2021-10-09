#include <vector>
#include "eigen3/Eigen/Dense"
#include "particle_filter.h"

using std::vector;

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

  // TODO: parallelize this loop
  for (Particle& p : particles_) {
    Update(ranges_sample, range_min, range_max, angle_min, sample_angle_max, p);
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
