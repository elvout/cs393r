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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "navigation.h"
#include <algorithm>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "visualization/visualization.h"

using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
}  // namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n)
    : odom_initialized_(false),
      localization_initialized_(false),
      robot_loc_(0, 0),
      robot_angle_(0),
      robot_vel_(0, 0),
      robot_omega_(0),
      nav_complete_(true),
      nav_start_loc_(0, 0),
      nav_goal_loc_(0, 0),
      nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  nav_start_loc_ = robot_loc_;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
  point_cloud_ = cloud;
}

void Navigation::Run() {
  // This function gets called `kUpdateFreqency` times a second to form the control loop.

  constexpr float kMaxSpeed = 1.0;
  constexpr float kMaxAccel = 4.0;
  constexpr float kMaxDecel = -4.0;

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_)
    return;

  if (nav_complete_) {
    return;
  }

  const float target_displacement = (nav_goal_loc_ - nav_start_loc_).norm();
  const float cur_displacement = (robot_loc_ - nav_start_loc_).norm();

  printf("[Navigation::Run]: %.2f m / %.2f m @ %.2f m/s\n", cur_displacement, target_displacement,
         robot_vel_.norm());

  const float v_peak = sqrt((2 * target_displacement * kMaxAccel * fabsf(kMaxDecel)) /
                            (kMaxAccel + fabsf(kMaxDecel)));

  if (v_peak > kMaxSpeed) {
    // case 1: there is enough distance for the car to reach its max speed

    // The current displacement thresholds for each TOC stage.
    const float stage_1_disp = kMaxSpeed * kMaxSpeed / (2 * kMaxAccel);
    const float stage_2_disp =
        target_displacement - (kMaxSpeed * kMaxSpeed / (2 * fabsf(kMaxDecel)));

    if (robot_vel_.norm() < kMaxSpeed && cur_displacement < stage_1_disp) {
      drive_msg_.velocity = std::min(kMaxSpeed, robot_vel_.norm() + kMaxAccel / kUpdateFrequency);
    } else if (cur_displacement < stage_2_disp) {
      // the car should be traveling at max velocity
      // stay at max velocity
    } else {
      drive_msg_.velocity = std::max(0.0f, robot_vel_.norm() + kMaxDecel / kUpdateFrequency);

      if (drive_msg_.velocity == 0) {
        nav_complete_ = true;
      }
    }
  } else {
    // case 2: there is not enough distance for the car to reach its max speed
    const float stage_1_disp = v_peak * v_peak / (2 * kMaxAccel);

    if (robot_vel_.norm() < v_peak && cur_displacement < stage_1_disp) {
      drive_msg_.velocity = std::min(v_peak, robot_vel_.norm() + kMaxAccel / kUpdateFrequency);
    } else {
      drive_msg_.velocity = std::max(0.0f, robot_vel_.norm() + kMaxDecel / kUpdateFrequency);

      if (drive_msg_.velocity == 0) {
        nav_complete_ = true;
      }
    }
  }

  // The latest observed point cloud is accessible via "point_cloud_"

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
