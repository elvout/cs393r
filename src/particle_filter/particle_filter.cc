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
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "models/motion.hh"
#include "models/sensor.hh"
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

// read configuration values from particle_filter.lua
CONFIG_DOUBLE(RobustSymmetricThreshold, "symmetric_threshold");
CONFIG_DOUBLE(Gamma, "sensor_model_gamma");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : prev_odom_loc_(0, 0), prev_odom_angle_(0), odom_initialized_(false) {}

const std::vector<Particle>& ParticleFilter::GetParticles() const {
  return particles_;
}

namespace {
const Eigen::Vector2f kLaserOffset(0.2, 0);

/**
 * Normalize the log-likelihood weights of all particles in place such
 * that the maximum log-likelihood weight is 0.
 */
void NormalizeParticles(std::vector<Particle>& particles) {
  constexpr double infinity = std::numeric_limits<double>::infinity();

  double max_particle_weight = -infinity;
  for (const Particle& p : particles) {
    max_particle_weight = std::max(max_particle_weight, p.weight);
  }

  if (max_particle_weight == 0) {
    return;
  } else if (max_particle_weight == -infinity) {
    for (Particle& p : particles) {
      p.weight = 0;
    }
  } else {
    for (Particle& p : particles) {
      p.weight -= max_particle_weight;
    }
  }
}

}  // namespace

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
 *  contains exactly `num_ranges` elements, some of which may be marked
 *  invalid, indicating that no intersections were found for the
 *  corresponding laser scans.
 */
std::vector<Observation> ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                                                const float angle,
                                                                const int num_ranges,
                                                                const float range_min,
                                                                const float range_max,
                                                                const float angle_min,
                                                                const float angle_max) const {
  constexpr float infinity = std::numeric_limits<float>::infinity();
  const auto laser_loc =
      std::make_shared<Eigen::Vector2f>(loc + Eigen::Rotation2Df(angle) * kLaserOffset);

  std::vector<Observation> point_cloud(num_ranges);
  std::vector<float> point_cloud_min_dist(num_ranges, infinity);
  std::vector<line2f> scan_lines(num_ranges);

  // For each laser scan, we want to find the closest line intersection within the
  // valid range interval.
  const float scan_res = (angle_max - angle_min) / (num_ranges - 1);
  const Eigen::Vector2f range_min_v(range_min, 0);
  const Eigen::Vector2f range_max_v(range_max, 0);

  // precompute scan lines
  for (int i = 0; i < num_ranges; i++) {
    const float scan_angle = angle_min + i * scan_res;
    const Eigen::Rotation2Df scan_rot(angle + scan_angle);

    const Eigen::Vector2f scan_start = *laser_loc + scan_rot * range_min_v;
    const Eigen::Vector2f scan_end = *laser_loc + scan_rot * range_max_v;
    const line2f scan_line(scan_start.x(), scan_start.y(), scan_end.x(), scan_end.y());
    scan_lines[i] = scan_line;

    point_cloud[i].msg_idx = i;
    point_cloud[i].laser_loc = laser_loc;
  }

  float __dummy_float;
  Eigen::Vector2f __dummy_point;
  Eigen::Vector2f intersection_point;
  for (const line2f& map_line : map_.lines) {
    // If the map line does not intersect with the a circle centered at
    // the laser with radius `range_max`, there is no line intersection.
    if (!geometry::FurthestFreePointCircle(map_line.p0, map_line.p1, *laser_loc, range_max,
                                           &__dummy_float, &__dummy_point)) {
      continue;
    }

    for (int i = 0; i < num_ranges; i++) {
      if (map_line.Intersection(scan_lines[i], &intersection_point)) {
        const float intersect_dist = (intersection_point - *laser_loc).norm();
        if (intersect_dist < point_cloud_min_dist[i]) {
          point_cloud[i].valid = true;
          point_cloud[i].obs_point = intersection_point;
          point_cloud[i].range = intersect_dist;
          point_cloud_min_dist[i] = intersect_dist;
        }
      }
    }
  }

  return point_cloud;
}

std::vector<Observation> ParticleFilter::DensitySampledPointCloud(
    const std::vector<Observation>& point_cloud) {
  if (point_cloud.empty()) {
    return point_cloud;
  }

  float range_sum = 0;
  float max_valid_range = 0;
  size_t valid_ranges = 0;

  for (const auto& point : point_cloud) {
    if (!point.valid) {
      continue;
    }

    valid_ranges++;
    range_sum += point.range;
    max_valid_range = std::max(max_valid_range, point.range);
  }

  const size_t target_sample_size = valid_ranges / 10;
  const double p_take_max = target_sample_size * max_valid_range / range_sum;

  std::vector<Observation> samples;
  samples.reserve(target_sample_size);

  for (const auto& point : point_cloud) {
    if (!point.valid) {
      continue;
    }

    const float p_take = point.range / max_valid_range * p_take_max;
    if (rng_.UniformRandom() <= p_take) {
      samples.push_back(point);
    }
  }

  // printf("[DensitySampledPointCloud]: # valid ranges: %lu\n", valid_ranges);
  // printf("[DensitySampledPointCloud]: target sample size: %lu\n", target_sample_size);
  // printf("[DensitySampledPointCloud]: actual sample size %lu\n", samples.size());
  return samples;
}

/**
 * Updates the weight of the particle in-place using the observation
 * likelihood model, the LIDAR sensor data, and the predicted point
 * cloud at the particle.
 *
 * The updated weight of the particle is a relative log-likelihood
 * value.
 *
 * Parameters:
 *  - ranges: An ordered (from angle_min to angle_max) list of sensor
 *      readings. If systematic sampling is desired, the sampled vector
 *      should be passed as the parameter.
 *  - range_min: The closest meaningful sensor value.
 *  - range_max: The farthest meaningful sensor value.
 *  - angle_min: The minimum laser scan angle. (ccw)
 *  - angle_max: The maximum laser scan angle. (ccw)
 *  - particle: The particle to update in-place.
 */
void ParticleFilter::Update(const vector<Observation>& ranges,
                            const size_t full_size,
                            const float range_min,
                            const float range_max,
                            const float angle_min,
                            const float angle_max,
                            Particle& particle) {
  double log_p = 0;
  const auto point_cloud = GetPredictedPointCloud(particle.loc, particle.angle, full_size,
                                                  range_min, range_max, angle_min, angle_max);
  const Eigen::Vector2f laser_loc =
      particle.loc + Eigen::Rotation2Df(particle.angle) * kLaserOffset;

  const double log_threshold =
      models::RobustLogSensorModelThreshold(CONFIG_RobustSymmetricThreshold);

  for (const auto& obs : ranges) {
    const double actual_range = static_cast<double>(obs.range);
    if (actual_range <= range_min || actual_range >= range_max) {
      continue;
    }

    int msg_idx = obs.msg_idx;

    if (!point_cloud[msg_idx].valid) {
      log_p += std::max(log_threshold, models::EvalLogSensorModel(actual_range, range_max - 1e-5));
    } else {
      const double predicted_range = static_cast<double>(point_cloud[msg_idx].range);
      log_p += std::max(log_threshold, models::EvalLogSensorModel(actual_range, predicted_range));
    }
  }

  particle.weight += CONFIG_Gamma * log_p;
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

  vector<Particle> new_particles;
  double running_sum = 0;

  static vector<double> cumulative_weights(FLAGS_num_particles);

  NormalizeParticles(particles_);

  std::priority_queue<double> debug_weights;
  for (const Particle& p : particles_) {
    debug_weights.push(p.weight);
  }

  printf("[ParticleFilter::Resample]: top 5 log-likelihoods:\n");
  for (size_t i = 0; !debug_weights.empty() && i < 5; i++) {
    printf("\t%f\n", debug_weights.top());
    debug_weights.pop();
  }

  // get cumulative sum
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    running_sum += exp(particles_[i].weight);
    cumulative_weights[i] = running_sum;
  }

  // pick initial random number and step with fixed step size
  const double step_size = running_sum / FLAGS_num_particles;
  double sample_point = rng_.UniformRandom(0, step_size);

  // Resample by stepping forward the pointer
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    while (cumulative_weights[i] > sample_point) {
      new_particles.emplace_back(particles_[i].loc, particles_[i].angle, 0.0);
      sample_point += step_size;
    }
  }

  particles_ = new_particles;
}

/**
 * This function predicts the new pose of each particle using the motion
 * model and new odometry data.
 */
void ParticleFilter::Predict(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  const Eigen::Vector2f odom_disp = odom_loc - prev_odom_loc_;
  const Eigen::Vector2f base_disp = Eigen::Rotation2Df(-prev_odom_angle_) * odom_disp;
  const float angular_disp = math_util::ReflexToConvexAngle(odom_angle - prev_odom_angle_);

  for (Particle& p : particles_) {
    const auto [noisy_disp, noisy_rotation] = models::SampleMotionModel(base_disp, angular_disp);

    // Rotate the translation to occur in the particle's frame.
    p.loc += Eigen::Rotation2Df(p.angle) * noisy_disp;
    p.angle += noisy_rotation;
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

  particles_.clear();
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    Vector2f noisy_loc(loc.x() + rng_.Gaussian(0, kNoiseStdDev),
                       loc.y() + rng_.Gaussian(0, kNoiseStdDev));
    float noisy_angle = rng_.Gaussian(angle, M_PI / 12);
    particles_.emplace_back(std::move(noisy_loc), noisy_angle, 0.0);
  }
}

/**
 * Computes the best estimate of the robot's location based on the
 * current set of particles.
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

  // normalize a copy to avoid concurrent modification
  std::vector<Particle> particles_copy(particles_);
  NormalizeParticles(particles_copy);

  for (const Particle& p : particles_copy) {
    const double linear_weight = std::exp(p.weight);
    mean_loc += p.loc * linear_weight;
    mean_angle_cos += std::cos(p.angle) * linear_weight;
    mean_angle_sin += std::sin(p.angle) * linear_weight;
    total_weight += linear_weight;
  }

  mean_loc /= total_weight;
  mean_angle_cos /= total_weight;
  mean_angle_sin /= total_weight;

  const double mean_angle = std::atan2(mean_angle_sin, mean_angle_cos);

  return std::make_pair(std::move(mean_loc), mean_angle);
}

}  // namespace particle_filter
