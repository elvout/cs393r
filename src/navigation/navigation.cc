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
#include <cmath>
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
      nav_goal_loc_(0, 0),
      nav_goal_angle_(0),
      nav_goal_disp_(0, 0),
      last_odom_pose_() {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {}

/**
 * Set the remaining displacement for navigation.
 */
void Navigation::SetNavDisplacement(const float dx, const float dy) {
  nav_goal_disp_ = {dx, dy};
  last_odom_pose_.Set(odom_angle_, odom_loc_);
  nav_complete_ = false;
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
  // This function gets called `kUpdateFrequency` times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_)
    return;

  if (nav_complete_) {
    return;
  }

  // Update the remaining displacement.
  // Eigen::Vector2f inst_disp = Eigen::Rotation2Df(odom_angle_).toRotationMatrix() * odom_loc_ -
  //   Eigen::Rotation2Df(last_odom_pose_.angle).toRotationMatrix() * last_odom_pose_.translation;
  Eigen::Vector2f inst_disp = odom_loc_ - last_odom_pose_.translation;
  Eigen::Vector2f corrected_disp = Eigen::Rotation2Df(-last_odom_pose_.angle) * inst_disp;
  float inst_angular_disp = odom_angle_ - last_odom_pose_.angle;
  nav_goal_disp_ -= corrected_disp;
  nav_goal_disp_ = Eigen::Rotation2Df(-inst_angular_disp).toRotationMatrix() * nav_goal_disp_;

  float curvature = 2 * nav_goal_disp_.y() / (nav_goal_disp_.dot(nav_goal_disp_));
  // TODO: edge case: curvature = 0

  float remaining_angular_disp = 2 * std::asin(curvature * nav_goal_disp_.norm() / 2);
  // arc length
  float remaining_distance = remaining_angular_disp / curvature;

  printf("[Navigation::Run]\n");
  printf("\todom loc: [%.2f, %.2f]\n", odom_loc_.x(), odom_loc_.y());
  printf("\theading: %.2fº\n", RadToDeg(odom_angle_));
  printf("\tinstantaneous displacement: [%.4f, %.4f]\n", inst_disp.x(), inst_disp.y());
  printf("\tinstantaneous displacement (corrected): [%.4f, %.4f]\n", corrected_disp.x(),
         corrected_disp.y());
  printf("\tinstantaneous angular difference: %.2fº\n", RadToDeg(inst_angular_disp));
  printf("\tcurvature: %.2f\n", curvature);
  printf("\tturning radius: %.2f\n", 1 / curvature);
  printf("\tremaining displacement: [%.2f, %.2f]\n", nav_goal_disp_.x(), nav_goal_disp_.y());
  printf("\tremaining angular displacement: %.2fº\n", RadToDeg(remaining_angular_disp));
  printf("\tremaining arc length: %.2f\n", remaining_distance);

  const float braking_distance = Sq(kMaxSpeed) / (2 * std::abs(kMaxDecel));
  const float cur_speed = robot_vel_.norm();
  if (remaining_distance <= braking_distance) {
    drive_msg_.velocity = std::max(0.0f, cur_speed + kMaxDecel / kUpdateFrequency);
  } else {
    drive_msg_.velocity = std::min(kMaxSpeed, cur_speed + kMaxAccel / kUpdateFrequency);
  }
  drive_msg_.curvature = curvature;

  last_odom_pose_.Set(odom_angle_, odom_loc_);

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
