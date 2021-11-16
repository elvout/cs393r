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

// Force `assert` statements to work.
// Some of the compiler flags in the shared libraries define NDEBUG.
// TODO: do this more gracefully
#ifdef NDEBUG
#undef NDEBUG
#endif

#include "navigation.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <type_traits>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "path_finding.hh"
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
      nav_graph_(25, vector_map::VectorMap(map_file)),
      nav_goal_disp_(0, 0),
      last_odom_pose_() {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  if (!localization_initialized_) {
    printf("[Navigation::SetNavGoal] error: localization not initialized.");
  } else {
    std::vector<Eigen::Vector2f> path = astar(nav_graph_, robot_loc_, loc);
  }
}

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

/**
 * Return the best path to the target to take based on point cloud data.
 *
 * The best path is defined as the path with minimal distace of closest
 * approach (if completely disjoint or externally tangent) or periapsis
 * (if intersecting) to the target.
 *
 * TODO: factor in clearance
 */
PathOption findBestPath(const std::vector<Vector2f>& point_cloud, const Vector2f& target) {
  constexpr float kWheelBase = 0.33;  // m
  const float min_steering_angle = atan(kWheelBase / -1.1);
  const float max_steering_angle = atan(kWheelBase / 1.1);
  constexpr size_t kNumSteps = 50;
  const float angle_step_size = (max_steering_angle - min_steering_angle) / kNumSteps;

  std::vector<PathOption> path_options;
  path_options.reserve(kNumSteps);

  for (size_t i = 0; i < kNumSteps; i++) {
    float steering_angle = min_steering_angle + angle_step_size * i;
    float curvature = tan(steering_angle) / kWheelBase;

    path_options.emplace_back(curvature, point_cloud, target);
  }

  PathOption& closest_approach_path = path_options[0];
  float min_dist_of_closest_approach =
      (closest_approach_path.closest_point_to_target - target).norm();

  for (const PathOption& path : path_options) {
    float dist_of_closest_approach = (path.closest_point_to_target - target).norm();
    if (dist_of_closest_approach < min_dist_of_closest_approach) {
      min_dist_of_closest_approach = dist_of_closest_approach;
      closest_approach_path = path;
    }

    path.visualize(local_viz_msg_, 0x000000);
  }

  // visualize the best path in green
  closest_approach_path.visualize(local_viz_msg_, 0x00ff00);
  return closest_approach_path;
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

  // Update the displacement target based on new odometry data.
  const Eigen::Vector2f odom_disp = odom_loc_ - last_odom_pose_.translation;
  const Eigen::Vector2f reference_disp = Eigen::Rotation2Df(-last_odom_pose_.angle) * odom_disp;
  const float inst_angular_disp = odom_angle_ - last_odom_pose_.angle;
  nav_goal_disp_ -= reference_disp;
  nav_goal_disp_ = Eigen::Rotation2Df(-inst_angular_disp) * nav_goal_disp_;
  last_odom_pose_.Set(odom_angle_, odom_loc_);

  // Transform predicted remaining displacement and point cloud
  // based on previous commands.
  Vector2f predicted_nav_goal_disp = nav_goal_disp_;
  std::vector<Vector2f> predicted_point_cloud = point_cloud_;
  for (const auto& msg : drive_msg_hist_) {
    const float arc_len = msg.velocity / kUpdateFrequency;
    const float turning_radius = 1 / msg.curvature;
    const float subtended_angle = arc_len / turning_radius;
    Eigen::Rotation2Df rot(-subtended_angle);

    Vector2f disp_vec(0, 0);
    if (msg.curvature == 0) {
      disp_vec.x() = arc_len;
    } else {
      disp_vec.x() = turning_radius * std::sin(subtended_angle);
      disp_vec.y() = turning_radius - turning_radius * std::cos(subtended_angle);
    }

    predicted_nav_goal_disp -= disp_vec;
    predicted_nav_goal_disp = rot * predicted_nav_goal_disp;

    for (auto& point : predicted_point_cloud) {
      point -= disp_vec;
      point = rot * point;
    }
  }

  for (auto& point : predicted_point_cloud) {
    visualization::DrawPoint(point, 0x000000, local_viz_msg_);
  }

  PathOption best_path = findBestPath(predicted_point_cloud, predicted_nav_goal_disp);

  float remaining_distance = best_path.free_path_length;

  const float braking_distance = Sq(kMaxSpeed) / (2 * std::abs(kMaxDecel));
  float cur_speed;
  if (drive_msg_hist_.empty()) {
    cur_speed = robot_vel_.norm();
  } else {
    cur_speed = drive_msg_hist_.back().velocity;
  }
  if (remaining_distance <= braking_distance) {
    drive_msg_.velocity = std::max(0.0f, cur_speed + kMaxDecel / kUpdateFrequency);

    if (drive_msg_.velocity == 0.0f) {
      nav_complete_ = true;
    }
  } else {
    drive_msg_.velocity = std::min(kMaxSpeed, cur_speed + kMaxAccel / kUpdateFrequency);
  }
  drive_msg_.curvature = best_path.curvature;

  drive_msg_hist_.emplace_back(drive_msg_);
  if (drive_msg_hist_.size() > kControlHistorySize) {
    drive_msg_hist_.pop_front();
  }

  printf("[Navigation::Run]\n");
  printf("\tinstantaneous displacement (odom): [%.4f, %.4f]\n", odom_disp.x(), odom_disp.y());
  printf("\tinstantaneous displacement (reference): [%.4f, %.4f]\n", reference_disp.x(),
         reference_disp.y());
  printf("\tinstantaneous angular difference: %.2fº\n", RadToDeg(inst_angular_disp));
  printf("\tcommand curvature: %.2f\n", drive_msg_.curvature);
  printf("\tcommand turning radius: %.2f\n", 1 / drive_msg_.curvature);
  printf("\tcommand speed: %.2f\n", drive_msg_.velocity);
  printf("\tremaining displacement: [%.2f, %.2f]\n", predicted_nav_goal_disp.x(),
         predicted_nav_goal_disp.y());
  printf("\tremaining arc length: %.2f\n", remaining_distance);

  // draw the target location
  visualization::DrawCross(nav_goal_disp_, 1, 0xff0000, local_viz_msg_);

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
