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
#include <queue>
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

// read configuration values from particle_filter.lua
CONFIG_DOUBLE(k1, "motion_model_k1");
CONFIG_DOUBLE(k2, "motion_model_k2");
CONFIG_DOUBLE(k3, "motion_model_k3");
CONFIG_DOUBLE(k4, "motion_model_k4");
CONFIG_DOUBLE(LidarStddev, "lidar_stddev");
CONFIG_DOUBLE(GaussianLowerBound, "sensor_model_d_short");
CONFIG_DOUBLE(GaussianUpperBound, "sensor_model_d_long");
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

double NormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2pi))
  constexpr double inv_sqrt2pi = 0.39894228040143267794;

  double z = (val - mean) / stddev;
  return inv_sqrt2pi / stddev * std::exp(-0.5 * z * z);
}

double NormalCdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2))
  constexpr double inv_sqrt2 = 0.70710678118654752440;

  double z = (val - mean) / stddev;
  return 0.5 + 0.5 * std::erf(z * inv_sqrt2);
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
void ParticleFilter::Update(const vector<float>& ranges,
                            const float range_min,
                            const float range_max,
                            const float angle_min,
                            const float angle_max,
                            Particle& particle) {
  double log_p = 0;
  const auto point_cloud = GetPredictedPointCloud(particle.loc, particle.angle, ranges.size(),
                                                  range_min, range_max, angle_min, angle_max);
  const Eigen::Vector2f laser_loc =
      particle.loc + Eigen::Rotation2Df(particle.angle) * kLaserOffset;

  for (size_t i = 0; i < ranges.size(); i++) {
    const double actual_range = static_cast<double>(ranges[i]);
    if (actual_range <= range_min || actual_range >= range_max) {
      continue;
    }

    double p_integral = 0;
    // Integral for the lower interval.
    p_integral += (actual_range + CONFIG_GaussianLowerBound - range_min) *
                  NormalPdf(CONFIG_GaussianLowerBound, 0, CONFIG_LidarStddev);
    // Integral for the Gaussian interval.
    p_integral += NormalCdf(CONFIG_GaussianUpperBound, 0, CONFIG_LidarStddev) -
                  NormalCdf(CONFIG_GaussianLowerBound, 0, CONFIG_LidarStddev);
    // Integral for the upper interval.
    p_integral += (range_max - (actual_range + CONFIG_GaussianUpperBound)) *
                  NormalPdf(CONFIG_GaussianUpperBound, 0, CONFIG_LidarStddev);

    // Calculate the linear probability since we need to normalize the value
    // with the integral.
    double p = 0;

    if (!point_cloud[i].has_value()) {
      // No intersection was found in the map file, but there is an
      // object observed by the actual LIDAR scanner. Since this could
      // be an object that was simply not included in the map (e.g. a chair),
      // incur a penalty based on the upper interval.

      p = NormalPdf(CONFIG_GaussianUpperBound, 0, CONFIG_LidarStddev);
    } else {
      const Eigen::Vector2f& predicted_point = *point_cloud[i];
      const double predicted_range = static_cast<double>((laser_loc - predicted_point).norm());

      if (predicted_range <= range_min || predicted_range >= range_max) {
        p = 0;
      } else if (predicted_range < actual_range + CONFIG_GaussianLowerBound) {
        p = NormalPdf(CONFIG_GaussianLowerBound, 0, CONFIG_LidarStddev);
      } else if (predicted_range > actual_range + CONFIG_GaussianUpperBound) {
        p = NormalPdf(CONFIG_GaussianUpperBound, 0, CONFIG_LidarStddev);
      } else {
        p = NormalPdf(predicted_range, actual_range, CONFIG_LidarStddev);
      }
    }

    p /= p_integral;
    log_p += std::log(p);
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

  const double translate_std = CONFIG_k1 * base_disp.norm() + CONFIG_k2 * std::abs(angular_disp);
  const double rotate_std = CONFIG_k3 * base_disp.norm() + CONFIG_k4 * std::abs(angular_disp);

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
