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

SLAM::SLAM() : prev_odom_loc_(0, 0), prev_odom_angle_(0), odom_initialized_(false) {}

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

  const float odom_disp = (prev_odom_loc_ - last_obs_odom_loc).norm();
  const float odom_angle_disp =
      std::abs(math_util::ReflexToConvexAngle(prev_odom_angle_ - last_obs_odom_angle));

  if (odom_disp >= kMinDispThreshold || odom_angle_disp >= kMinAngleDispThreshold) {
    // TODO
  } else {
    return;
  }

  // Overwrite last_obs_odom_[loc,angle].
  // Create copies beforehand? It's possible for an odometry observation
  // to occur concurrently.
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  // Keep track of odometry to estimate how far the robot has moved between
  // poses.
  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
  odom_initialized_ = true;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
