//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;

  Particle() = default;
  Particle(Eigen::Vector2f loc, float angle, double weight)
      : loc(std::move(loc)), angle(angle), weight(weight) {}
};

class ParticleFilter {
 public:
  // Default Constructor.
  ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc, const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file, const Eigen::Vector2f& loc, const float angle);

  // Return the list of particles.
  const std::vector<Particle>& GetParticles() const;

  // Get robot's current location.
  std::pair<Eigen::Vector2f, float> GetLocation() const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  // For debugging: get predicted point cloud from current location.
  std::vector<std::optional<Eigen::Vector2f>> GetPredictedPointCloud(const Eigen::Vector2f& loc,
                                                                     const float angle,
                                                                     const int num_ranges,
                                                                     const float range_min,
                                                                     const float range_max,
                                                                     const float angle_min,
                                                                     const float angle_max) const;

 private:
  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
};
}  // namespace particle_filter

#endif  // SRC_PARTICLE_FILTER_H_
