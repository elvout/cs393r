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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

DEFINE_uint32(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : prev_odom_loc_(0, 0), prev_odom_angle_(0), odom_initialized_(false) {}

const std::vector<Particle>& ParticleFilter::GetParticles() const {
  return particles_;
}

/**
 * Computes what the predicted point cloud (sensor readings) would be
 * if the car was at the pose (loc, angle) with sensor characteristics
 * defined by the remaining parameters.
 *
 * This function is used as part of the Observation Likelihood Model and
 * Update step.
 *
 * Parameters:
 *  - loc: the location of the base link of the car in the map frame
 *  - angle: the angle of the base link of the car in the map frame
 *  - num_ranges: the number of individual laser readings
 *  - range_min: the closest meaningful reading value
 *  - range_max: the farthest meaningful reading value
 *  - angle_min: the minimum laser scan angle (ccw)
 *  - angle_max: the maximum laser scan angle (ccw)
 *
 * Returns:
 *  A std::vector containing a point cloud in the map frame. The vector
 *  contains exactly `num_ranges` elements, some of which may be empty,
 *  indicating that no intersections were found for the corresponding
 *  laser scans.
 */
std::vector<std::optional<Eigen::Vector2f>> ParticleFilter::GetPredictedPointCloud(
    const Vector2f& loc,
    const float angle,
    const int num_ranges,
    const float range_min,
    const float range_max,
    const float angle_min,
    const float angle_max) const {
  static const Eigen::Vector2f kLaserOffset(0.2, 0);
  const Eigen::Vector2f laser_loc = loc + Eigen::Rotation2Df(angle) * kLaserOffset;

  std::vector<std::optional<Eigen::Vector2f>> point_cloud;
  point_cloud.reserve(num_ranges);

  // For each laser scan, we want to find the closest line intersection within the
  // valid range interval.
  const float scan_res = (angle_max - angle_min) / num_ranges;
  for (int i = 0; i < num_ranges; i++) {
    const float scan_angle = angle_min + i * scan_res;

    float closest_intersect_dist = range_max + 1;
    std::optional<Eigen::Vector2f> closest_intersect_point;

    const Eigen::Vector2f scan_start =
        laser_loc + Eigen::Rotation2Df(angle + scan_angle) * Eigen::Vector2f(range_min, 0);
    const Eigen::Vector2f scan_end =
        laser_loc + Eigen::Rotation2Df(angle + scan_angle) * Eigen::Vector2f(range_max, 0);
    const line2f scan_line(scan_start.x(), scan_start.y(), scan_end.x(), scan_end.y());

    Eigen::Vector2f intersection_point;  // this could probably be static
    for (const line2f& map_line : map_.lines) {
      if (map_line.Intersection(scan_line, &intersection_point)) {
        const float intersect_dist = (intersection_point - laser_loc).norm();
        if (intersect_dist < closest_intersect_dist) {
          closest_intersect_point = intersection_point;
          closest_intersect_dist = intersect_dist;
        }
      }
    }

    if (closest_intersect_point.has_value()) {
      point_cloud.push_back(std::move(*closest_intersect_point));
    } else {
      point_cloud.emplace_back();
    }
  }

  return point_cloud;
}

/**
 * TODO: doc
 * IMPORTANT: assigns the weight of the particle to a relative measurement
 *  of its log-likelihood.
 *
 *
 * TODO: do we call this function NUM_PARTICLES times or should we reweight
 *  every particle in this function? The existing API suggests that we're only
 *  updating a signle particle.
 *
 */
void ParticleFilter::Update(const vector<float>& ranges,
                            const float range_min,
                            const float range_max,
                            const float angle_min,
                            const float angle_max,
                            Particle& particle) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  constexpr double kLidarStddev = 0.1;  // meters, inflated
  constexpr double kLidarVar = kLidarStddev * kLidarStddev;

  double log_p = 0;
  const auto point_cloud = GetPredictedPointCloud(particle.loc, particle.angle, ranges.size(),
                                                  range_min, range_max, angle_min, angle_max);

  // ranges should be sampled prior to calling this function
  for (size_t i = 0; i < ranges.size(); i++) {
    float actual_range = ranges[i];
    if (actual_range <= range_min || actual_range >= range_max) {
      continue;
    }

    double range_diff = 0;
    if (point_cloud[i].has_value()) {
      range_diff = (particle.loc - *point_cloud[i]).norm() - actual_range;
    } else {
      // Incur a penalty using the range itself.
      // TODO: does this need to be tuned?
      range_diff = actual_range;
    }

    log_p += -(range_diff * range_diff) / kLidarVar;
  }

  particle.weight = log_p;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable.
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling:
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n", x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  const float range_min,
                                  const float range_max,
                                  const float angle_min,
                                  const float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // Sample the sensor readings before computing point cloud estimations
  // to improve performance.
  std::vector<float> ranges_sample;
  ranges_sample.reserve(ranges.size() / 10);
  for (size_t i = 0; i < ranges.size(); i += 10) {
    ranges_sample.push_back(ranges[i]);
  }
  const float sample_angle_max =
      angle_max - (angle_max - angle_min) / ranges.size() * (ranges.size() % 10);

  std::vector<Particle> particles_copy(particles_);
  for (Particle& p : particles_copy) {
    Update(ranges_sample, range_min, range_max, angle_min, sample_angle_max, p);
  }

  std::sort(particles_copy.begin(), particles_copy.end(),
            [](const Particle& a, const Particle& b) { return a.weight > b.weight; });

  // normalization step 1: normalize log-likelihoods using the maximum log-likelihood
  for (Particle& p : particles_copy) {
    p.weight -= particles_copy.front().weight;
  }
}

/**
 * This function predicts the new pose of each particle using the motion
 * model and new odometry data.
 */
void ParticleFilter::Predict(const Vector2f& odom_loc, const float odom_angle) {
  constexpr double k_1 = 0.5;  // error in translation from translation
  constexpr double k_2 = 0.5;  // error in translation from rotation
  constexpr double k_3 = 0.5;  // error in rotation from translation
  constexpr double k_4 = 0.5;  // error in rotation from rotation

  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  const Eigen::Vector2f odom_disp = odom_loc - prev_odom_loc_;
  const Eigen::Vector2f base_disp = Eigen::Rotation2Df(-prev_odom_angle_) * odom_disp;
  const float angular_disp = odom_angle - prev_odom_angle_;

  const double translate_std = k_1 * base_disp.norm() + k_2 * std::abs(angular_disp);
  const double rotate_std = k_3 * base_disp.norm() + k_4 * std::abs(angular_disp);

  for (Particle& p : particles_) {
    const Eigen::Vector2f translate_err(rng_.Gaussian(0, translate_std),
                                        rng_.Gaussian(0, translate_std));
    const double rotate_err = rng_.Gaussian(0, rotate_std);

    // Rotate the translation to occur in the particle's frame.
    p.loc += Eigen::Rotation2Df(p.angle) * (base_disp + translate_err);
    p.angle += angular_disp + rotate_err;
  }

  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file, const Vector2f& loc, const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  constexpr double kNoiseStdDev = 0.1;  // meters
  const double init_weight = 1.0 / FLAGS_num_particles;

  particles_.clear();
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    Vector2f noisy_loc(loc.x() + rng_.Gaussian(0, kNoiseStdDev),
                       loc.y() + rng_.Gaussian(0, kNoiseStdDev));
    particles_.emplace_back(std::move(noisy_loc), angle, init_weight);
  }
}

/**
 * Computes the best estimate of the robot's location based on the
 * current set of particles.
 *
 * CRITICAL: assumes particle weights are not log-likelihoods.
 */
std::pair<Eigen::Vector2f, float> ParticleFilter::GetLocation() const {
  // Compute the weighted mean of all the particles.

  // TODO: The mean is potentially suboptimal if the distribution of peaks
  //  is multimodal. In class suggestion: cluster the points and find the
  //  weighted mean of one of the clusters.

  Vector2f mean_loc(0, 0);
  double mean_angle_cos = 0;
  double mean_angle_sin = 0;
  double total_weight = 0;

  for (const Particle& p : particles_) {
    mean_loc += p.loc * p.weight;
    mean_angle_cos += std::cos(p.angle) * p.weight;
    mean_angle_sin += std::sin(p.angle) * p.weight;
    total_weight += p.weight;
  }

  mean_loc /= total_weight;
  mean_angle_cos /= total_weight;
  mean_angle_sin /= total_weight;

  const double mean_angle = std::atan2(mean_angle_sin, mean_angle_cos);

  return std::make_pair(std::move(mean_loc), mean_angle);
}

}  // namespace particle_filter
