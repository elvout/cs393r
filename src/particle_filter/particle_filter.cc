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

std::vector<Eigen::Vector2f> ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                                                    const float angle,
                                                                    const int num_ranges,
                                                                    const float range_min,
                                                                    const float range_max,
                                                                    const float angle_min,
                                                                    const float angle_max) const {
  std::vector<Vector2f> scan;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4);  // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    // printf("P0: %f, %f P1: %f,%f\n", my_line.p0.x(), my_line.p0.y(), my_line.p1.x(),
    //        my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point;  // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      // printf("Intersects at %f,%f\n", intersection_point.x(), intersection_point.y());
    } else {
      // printf("No intersection\n");
    }
  }

  return scan;
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
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
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
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
 */
std::pair<Eigen::Vector2f, float> ParticleFilter::GetLocation() const {
  // Compute the mean of all the particles.
  // TODO: the computation is surely more complex than this

  Vector2f mean_loc(0, 0);
  double mean_angle = 0;

  for (const Particle& p : particles_) {
    mean_loc += p.loc;
    mean_angle += p.angle;
  }

  mean_loc /= particles_.size();
  mean_angle /= particles_.size();

  return std::make_pair(std::move(mean_loc), mean_angle);
}

}  // namespace particle_filter
