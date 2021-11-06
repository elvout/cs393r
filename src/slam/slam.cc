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
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "models.hh"
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

namespace slam {

std::vector<Eigen::Vector2f> SLAMBelief::correlated_points(const RasterMap& prev_ref_map) {
  std::vector<Eigen::Vector2f> obs_points = PointsFromScan(obs);
  std::vector<Eigen::Vector2f> correlations;

  const auto [bel_disp, bel_rot] = belief_lookup.max_belief();
  const Eigen::Rotation2Df dtheta_rot(bel_rot);
  for (const Eigen::Vector2f& obs_point : obs_points) {
    // Translate the observation point into the reference frame.
    const Eigen::Vector2f query_point = dtheta_rot * obs_point + bel_disp;

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
      offline_eval_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
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
    init_bel.ref_loc = prev_odom_loc_;
    init_bel.ref_angle = prev_odom_angle_;
    init_bel.ref_map.eval(init_bel.obs);

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

  SLAMBelief bel;
  bel.odom_disp = Eigen::Rotation2Df(-last_obs_odom_angle) * (prev_odom_loc_ - last_obs_odom_loc);
  bel.odom_angle_disp = math_util::ReflexToConvexAngle(prev_odom_angle_ - last_obs_odom_angle);
  last_obs_odom_loc = prev_odom_loc_;
  last_obs_odom_angle = prev_odom_angle_;

  bel.obs = obs;

  bel.ref_loc = prev_odom_loc_;
  bel.ref_angle = prev_odom_angle_;
  // bel.ref_map.eval(bel.obs);

  // bel.belief_lookup.eval(belief_history.back().ref_map, bel.odom_disp, bel.odom_angle_disp,
  // bel.obs);

  printf("Odometry reported disp: [%.4f, %.4f] %.2fº\n", bel.odom_disp.x(), bel.odom_disp.y(),
         math_util::RadToDeg(bel.odom_angle_disp));

  // auto [max_disp, max_angle_disp] = bel.belief_lookup.max_belief();
  // printf("Max likelihood disp: [%.4f, %.4f] %.2fº\n", max_disp.x(), max_disp.y(),
  // math_util::RadToDeg(max_angle_disp));

  belief_history.push_back(bel);
  // exit(1);
}

void SLAM::OfflineBelEvaluation() {
  offline_eval_ = true;

  for (size_t i = 1; i < belief_history.size(); i++) {
    SLAMBelief& bel = belief_history[i];

    printf("Odometry reported disp: [%.4f, %.4f] %.2fº\n", bel.odom_disp.x(), bel.odom_disp.y(),
           math_util::RadToDeg(bel.odom_angle_disp));

    bel.ref_map.eval(bel.obs);
    bel.belief_lookup.eval(belief_history[i - 1].ref_map, bel.odom_disp, bel.odom_angle_disp,
                           bel.obs);

    auto [max_disp, max_angle_disp] = bel.belief_lookup.max_belief();
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

// use a raster map?
vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.

  if (!offline_eval_) {
    return map;
  }

  Eigen::Vector2f aggregate_disp(0, 0);
  double aggregate_rot = 0;

  for (int i = 1; i < belief_history.size(); i++) {
    SLAMBelief& bel = belief_history[i];

    auto [bel_disp, bel_rot] = bel.belief_lookup.max_belief();
    aggregate_disp += Eigen::Rotation2Df(aggregate_rot) * bel_disp;
    aggregate_rot = math_util::ConstrainAngle(aggregate_rot + bel_rot);

    std::vector<Eigen::Vector2f> points = PointsFromScan(bel.obs);
    std::vector<Eigen::Vector2f> corrs = bel.correlated_points(belief_history[i - 1].ref_map);
    printf("[SLAM::GetMap INFO] points: %lu, correlations: %lu\n", points.size(), corrs.size());
    for (const Eigen::Vector2f& point : corrs) {
      map.push_back(Rotation2Df(aggregate_rot) * point + aggregate_disp);
    }
  }

  return map;
}

}  // namespace slam
