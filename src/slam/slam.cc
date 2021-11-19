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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "slam.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include "common/common.hh"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "models/sensor.hh"
#include "sensor_msgs/LaserScan.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace {
const int global_tx_windowsize = 96;
const int coarse_tx_resolution = 24;
const int fine_tx_resolution = 4;

const int global_rot_windowsize = 45;
const int global_rot_resolution = 1;
}  // namespace

namespace slam {

SLAMBelief::SLAMBelief()
    : odom_disp(),
      odom_angle_disp(),
      obs(),
      belief_disp(),
      belief_angle_disp(),
      belief_loc(),
      belief_angle(),
      coarse_ref_map(coarse_tx_resolution),
      fine_ref_map(fine_tx_resolution) {}

std::vector<Eigen::Vector2f> SLAMBelief::correlated_points(const RasterMap& prev_ref_map) const {
  std::vector<Eigen::Vector2f> obs_points = models::PointsFromScan(obs);
  std::vector<Eigen::Vector2f> correlations;

  const Eigen::Rotation2Df dtheta_rot(belief_angle_disp);
  for (const Eigen::Vector2f& obs_point : obs_points) {
    // Translate the observation point into the reference frame.
    const Eigen::Vector2f query_point = dtheta_rot * obs_point + belief_disp;

    const float query_dist = query_point.norm();
    if (query_dist <= obs.range_min || query_dist >= obs.range_max) {
      continue;
    }

    const double log_obs_prob = prev_ref_map.query(query_point.x(), query_point.y());
    if (log_obs_prob != -std::numeric_limits<double>::infinity()) {
      correlations.push_back(obs_point);
    }
  }

  return correlations;
}

SLAM::SLAM()
    : prev_odom_loc_(0, 0),
      prev_odom_angle_(0),
      odom_initialized_(false),
      belief_history(),
      offline_eval_(false),
      map_(2) {}

std::pair<Eigen::Vector2f, float> SLAM::GetPose() const {
  if (belief_history.empty()) {
    return std::make_pair(Eigen::Vector2f(0, 0), 0);
  }

  return std::make_pair(belief_history.back().belief_loc, belief_history.back().belief_angle);
}

void SLAM::ObserveLaser(const sensor_msgs::LaserScan& obs) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  constexpr double kMinDispThreshold = 0.5;                // meters
  constexpr double kMinAngleDispThreshold = 0.5235987756;  // 30 degrees

  static Eigen::Vector2f last_obs_odom_loc(0, 0);
  static float last_obs_odom_angle = 0.0f;
  static bool bel_initialized = false;

  if (!odom_initialized_) {
    return;
  }

  if (!bel_initialized) {
    SLAMBelief init_bel;
    init_bel.obs = obs;
    init_bel.coarse_ref_map.eval(init_bel.obs);
    init_bel.fine_ref_map.eval(init_bel.obs);

    last_obs_odom_loc = prev_odom_loc_;
    last_obs_odom_angle = prev_odom_angle_;
    belief_history.push_back(std::move(init_bel));
    bel_initialized = true;
    return;
  }

  const float odom_disp = (prev_odom_loc_ - last_obs_odom_loc).norm();
  const float odom_angle_disp =
      std::abs(math_util::ReflexToConvexAngle(prev_odom_angle_ - last_obs_odom_angle));

  if (odom_disp < kMinDispThreshold && odom_angle_disp < kMinAngleDispThreshold) {
    return;
  }

  auto __delayedfn = common::runtime_dist.auto_lap("SLAM::ObserveLaser");

  SLAMBelief bel;
  bel.odom_disp = Eigen::Rotation2Df(-last_obs_odom_angle) * (prev_odom_loc_ - last_obs_odom_loc);
  bel.odom_angle_disp = math_util::ReflexToConvexAngle(prev_odom_angle_ - last_obs_odom_angle);
  last_obs_odom_loc = prev_odom_loc_;
  last_obs_odom_angle = prev_odom_angle_;

  bel.obs = obs;

  // Execute these tasks in the background.
  // RasterMap::eval generally runs faster than BeliefCube::eval, but this could
  // potentially cause a segfault if `bel` goes out of scope. (fix: wrap & await)
  common::thread_pool.put(std::bind(&RasterMap::eval, &bel.fine_ref_map, bel.obs));
  common::thread_pool.put(std::bind(&RasterMap::eval, &bel.coarse_ref_map, bel.obs));

  BeliefCube coarse_cube(global_tx_windowsize, coarse_tx_resolution, global_rot_windowsize,
                         global_rot_resolution);
  BeliefCube fine_cube(global_tx_windowsize, fine_tx_resolution, global_rot_windowsize,
                       global_rot_resolution);

  coarse_cube.eval(belief_history.back().coarse_ref_map, bel.odom_disp, bel.odom_angle_disp,
                   bel.obs, true, true);
  fine_cube.eval_with_coarse(belief_history.back().fine_ref_map, bel.odom_disp,
                             bel.odom_angle_disp, bel.obs, coarse_cube);

  const std::pair<Eigen::Vector2f, double> max_prob_belief = fine_cube.max_belief();
  bel.belief_disp = max_prob_belief.first;
  bel.belief_angle_disp = max_prob_belief.second;

  bel.belief_loc = belief_history.back().belief_loc +
                   Eigen::Rotation2Df(belief_history.back().belief_angle) * bel.belief_disp;
  bel.belief_angle =
      math_util::ReflexToConvexAngle(belief_history.back().belief_angle + bel.belief_angle_disp);

  printf("Odometry reported disp: [%.4f, %.4f] %.2fº\n", bel.odom_disp.x(), bel.odom_disp.y(),
         math_util::RadToDeg(bel.odom_angle_disp));

  printf("Max likelihood disp: [%.4f, %.4f] %.2fº\n", bel.belief_disp.x(), bel.belief_disp.y(),
         math_util::RadToDeg(bel.belief_angle_disp));

  belief_history.push_back(std::move(bel));
}

void SLAM::OfflineBelEvaluation() {
  offline_eval_ = true;

  for (size_t i = 1; i < belief_history.size(); i++) {
    SLAMBelief& bel = belief_history[i];

    printf("Odometry reported disp: [%.4f, %.4f] %.2fº\n", bel.odom_disp.x(), bel.odom_disp.y(),
           math_util::RadToDeg(bel.odom_angle_disp));

    BeliefCube coarse_cube(global_tx_windowsize, coarse_tx_resolution, global_rot_windowsize,
                           global_rot_resolution);
    BeliefCube fine_cube(global_tx_windowsize, fine_tx_resolution, global_rot_windowsize,
                         global_rot_resolution);

    coarse_cube.eval(belief_history[i - 1].coarse_ref_map, bel.odom_disp, bel.odom_angle_disp,
                     bel.obs, true, true);
    fine_cube.eval_with_coarse(belief_history[i - 1].fine_ref_map, bel.odom_disp,
                               bel.odom_angle_disp, bel.obs, coarse_cube);

    bel.coarse_ref_map.eval(bel.obs);
    bel.fine_ref_map.eval(bel.obs);

    auto [max_disp, max_angle_disp] = fine_cube.max_belief();
    bel.belief_disp = max_disp;
    bel.belief_angle_disp = max_angle_disp;
    printf("Max likelihood disp: [%.4f, %.4f] %.2fº\n", max_disp.x(), max_disp.y(),
           math_util::RadToDeg(math_util::ReflexToConvexAngle(max_angle_disp)));
    printf("------------\n");
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  // Keep track of odometry to estimate how far the robot has moved between
  // poses.
  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
  odom_initialized_ = true;
}

vector<Vector2f> SLAM::GetMap() {
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  static size_t last_update_idx = 0;

  auto __delayedfn = common::runtime_dist.auto_lap("SLAM::GetMap");

  size_t i = last_update_idx + 1;
  for (; i < belief_history.size(); i++) {
    const SLAMBelief& bel = belief_history[i];

    std::vector<Eigen::Vector2f> points = models::PointsFromScan(bel.obs);
    std::vector<Eigen::Vector2f> corrs = bel.correlated_points(belief_history[i - 1].fine_ref_map);
    printf("[SLAM::GetMap INFO] points: %lu, correlations: %lu\n", points.size(), corrs.size());
    const Eigen::Rotation2Df bel_rot(bel.belief_angle);
    for (const Eigen::Vector2f& point : corrs) {
      map_.add_coord(bel_rot * point + bel.belief_loc);
    }
  }
  last_update_idx = i - 1;

  return map_.export_coord_map();
}

}  // namespace slam
