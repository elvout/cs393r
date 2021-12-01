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
#include <memory>
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

SLAMBelief::SLAMBelief(const Eigen::Vector2f& prev_odom_loc,
                       const float prev_odom_angle,
                       const Eigen::Vector2f& odom_loc,
                       const float odom_angle,
                       const sensor_msgs::LaserScan& obs)
    : odom_disp(Eigen::Rotation2Df(-prev_odom_angle) * (odom_loc - prev_odom_loc)),
      odom_angle_disp(math_util::ReflexToConvexAngle(odom_angle - prev_odom_angle)),
      obs(obs) {
  auto fine_promise = std::make_shared<std::promise<RasterMap>>();
  auto coarse_promise = std::make_shared<std::promise<RasterMap>>();
  this->fine_ref_map = fine_promise->get_future().share();
  this->coarse_ref_map = coarse_promise->get_future().share();

  common::thread_pool.put([this, fine_promise] {
    auto __delayedfn = common::runtime_dist.auto_lap("Fine RasterMap");
    RasterMap fine_map(fine_tx_resolution);
    fine_map.eval(this->obs);
    fine_promise->set_value(std::move(fine_map));
  });

  common::thread_pool.put([this, coarse_promise] {
    auto __delayedfn = common::runtime_dist.auto_lap("Coarse RasterMap");
    RasterMap coarse_map(coarse_tx_resolution);
    coarse_map.eval(this->obs);
    coarse_promise->set_value(std::move(coarse_map));
  });
}

std::vector<Eigen::Vector2f> SLAMBelief::correlated_points(const RasterMap& prev_ref_map) const {
  std::vector<Eigen::Vector2f> correlations;

  const Eigen::Rotation2Df dtheta_rot(belief_angle_disp);
  for (Eigen::Index col_i = 0; col_i < obs.point_cloud().cols(); col_i++) {
    const Eigen::Vector2f& obs_point = obs.point_cloud().col(col_i);
    // Translate the observation point into the reference frame.
    const Eigen::Vector2f query_point = dtheta_rot * obs_point + belief_disp;

    const float query_dist = query_point.norm();
    if (query_dist <= obs.min_range_dist_ || query_dist >= obs.max_range_dist_) {
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
      map_(2) {}

std::pair<Eigen::Vector2f, float> SLAM::GetPose() const {
  if (belief_history.empty()) {
    return std::make_pair(Eigen::Vector2f(0, 0), 0);
  }

  return std::make_pair(belief_history.back()->belief_loc, belief_history.back()->belief_angle);
}

void SLAM::ObserveLaser(const sensor_msgs::LaserScan& scan_msg) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  constexpr double kMinDispThreshold = 0.5;                // meters
  constexpr double kMinAngleDispThreshold = 0.5235987756;  // 30 degrees

  static Eigen::Vector2f last_obs_odom_loc(0, 0);
  static float last_obs_odom_angle = 0.0f;

  if (!odom_initialized_) {
    return;
  }

  if (belief_history.empty()) {
    auto init_bel = std::make_shared<SLAMBelief>(last_obs_odom_loc, last_obs_odom_angle,
                                                 prev_odom_loc_, prev_odom_angle_, scan_msg);
    init_bel->belief_disp = Eigen::Vector2f(0, 0);
    init_bel->belief_angle_disp = 0;
    init_bel->belief_loc = Eigen::Vector2f(0, 0);
    init_bel->belief_angle = 0;
    belief_history.push_back(init_bel);

    last_obs_odom_loc = prev_odom_loc_;
    last_obs_odom_angle = prev_odom_angle_;
    return;
  }

  const float odom_disp = (prev_odom_loc_ - last_obs_odom_loc).norm();
  const float odom_angle_disp =
      std::abs(math_util::ReflexToConvexAngle(prev_odom_angle_ - last_obs_odom_angle));

  if (odom_disp < kMinDispThreshold && odom_angle_disp < kMinAngleDispThreshold) {
    return;
  }

  auto __delayedfn = common::runtime_dist.auto_lap("SLAM::ObserveLaser");

  auto bel = std::make_shared<SLAMBelief>(last_obs_odom_loc, last_obs_odom_angle, prev_odom_loc_,
                                          prev_odom_angle_, scan_msg);
  auto prev_bel = belief_history.back();
  last_obs_odom_loc = prev_odom_loc_;
  last_obs_odom_angle = prev_odom_angle_;

  BeliefCube coarse_cube(global_tx_windowsize, coarse_tx_resolution, global_rot_windowsize,
                         global_rot_resolution);
  BeliefCube fine_cube(global_tx_windowsize, fine_tx_resolution, global_rot_windowsize,
                       global_rot_resolution);

  const models::Observations sampled_obs = bel->obs.density_aware_sample(0.1);
  coarse_cube.eval(prev_bel->coarse_ref_map.get(), bel->odom_disp, bel->odom_angle_disp,
                   sampled_obs, true, true);
  fine_cube.eval_with_coarse(prev_bel->fine_ref_map.get(), bel->odom_disp, bel->odom_angle_disp,
                             sampled_obs, coarse_cube);

  const std::pair<Eigen::Vector2f, double> max_prob_belief = fine_cube.max_belief();
  bel->belief_disp = max_prob_belief.first;
  bel->belief_angle_disp = max_prob_belief.second;

  bel->belief_loc =
      prev_bel->belief_loc + Eigen::Rotation2Df(prev_bel->belief_angle) * bel->belief_disp;
  bel->belief_angle =
      math_util::ReflexToConvexAngle(prev_bel->belief_angle + bel->belief_angle_disp);

  printf("Odometry reported disp: [%.4f, %.4f] %.2fº\n", bel->odom_disp.x(), bel->odom_disp.y(),
         math_util::RadToDeg(bel->odom_angle_disp));

  printf("Max likelihood disp: [%.4f, %.4f] %.2fº\n", bel->belief_disp.x(), bel->belief_disp.y(),
         math_util::RadToDeg(bel->belief_angle_disp));

  belief_history.push_back(bel);
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
    std::shared_ptr<SLAMBelief> bel = belief_history[i];

    std::vector<Eigen::Vector2f> corrs =
        bel->correlated_points(belief_history[i - 1]->fine_ref_map.get());
    const Eigen::Rotation2Df bel_rot(bel->belief_angle);
    for (const Eigen::Vector2f& point : corrs) {
      map_.add_coord(bel_rot * point + bel->belief_loc);
    }
  }
  last_update_idx = i - 1;

  return map_.export_coord_map();
}

}  // namespace slam
