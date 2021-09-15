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
  nav_curvature_ = 2 * dy / (Sq(dx) + Sq(dy));
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

float angleAlongTurningPoint(Eigen::Vector2f a, Eigen::Vector2f turning_point_local) {
  // Unit circle typaclly starts at 1, 0 but ours starts at 0, -1
  auto norm = (a - turning_point_local).normalized();
  Eigen::Rotation2Df rot(M_PI / 2);
  norm = rot * norm;
  int s = Sign(norm.y());
  auto angle = acos(norm.x());
  if (s == -1) {
    angle = 2 * M_PI - angle;
  }
  return angle;
}

void Navigation::maxDistanceTravelable(float r,
                                       std::vector<Eigen::Vector2f> point_cloud_,
                                       PathOption& path_option) {
  // This function, given a turning radius and an arbitrary point cloud,
  // returns the maximum distance travelable on an arc with the current positioning of the cart

  // #define DEBUG_OD 1

  Eigen::Vector2f turning_point_local(0, r);
#ifdef DEBUG_OD
  visualization::DrawCross(turning_point_local, .1, 2, local_viz_msg_);
  visualization::DrawCross(this->robot_loc_, .3, 2, local_viz_msg_);
#endif

  float base_to_front = .42 * 1.25;
  float base_to_side = .14 * 1.25;

  // Using cardinal directions in car reference frame
  Eigen::Vector2f front_north_vector_local(base_to_front, base_to_side);
  Eigen::Vector2f front_south_vector_local(base_to_front, -base_to_side);
  Eigen::Vector2f side_west_vector_local(-base_to_front, base_to_side);
  Eigen::Vector2f side_east_vector_local(base_to_front, base_to_side);

#ifdef DEBUG_OD
  visualization::DrawLine(side_west_vector_local, side_east_vector_local, 5, local_viz_msg_);
  visualization::DrawLine(front_south_vector_local, front_north_vector_local, 5, local_viz_msg_);
#endif

  float front_north_radius = (turning_point_local - front_north_vector_local).norm();
  float front_south_radius = (turning_point_local - front_south_vector_local).norm();
  float side_west_radius = (turning_point_local - side_west_vector_local).norm();
  float side_east_radius = (turning_point_local - side_east_vector_local).norm();

#ifdef DEBUG_OD
  visualization::DrawArc(turning_point_local, front_north_radius, 0, 2 * M_PI, 1, local_viz_msg_);
  visualization::DrawArc(turning_point_local, front_south_radius, 0, 2 * M_PI, 1, local_viz_msg_);
  visualization::DrawArc(turning_point_local, side_west_radius, 0, 2 * M_PI, 1, local_viz_msg_);
  visualization::DrawArc(turning_point_local, side_east_radius, 0, 2 * M_PI, 1, local_viz_msg_);
#endif

  float max_angle_should_travel =
      angleAlongTurningPoint(this->nav_goal_disp_, turning_point_local);

  float min_alpha = 2 * M_PI;
  // TODO: What to do with this when there's no collision?
  Eigen::Vector2f min_alpha_point;

  for (auto point : point_cloud_) {
    visualization::DrawPoint(point, 4, local_viz_msg_);

    float point_radius = (turning_point_local - point).norm();

    // Front hit case
    bool front_collision = front_north_radius < point_radius && point_radius < front_south_radius;

    if (front_collision) {
      float collision_angle_total = angleAlongTurningPoint(point, turning_point_local);

      // From brainstorm scan
      float d = (turning_point_local - point).norm();
      float p = (d - front_north_radius) / (front_south_radius - front_north_radius);

      Eigen::Vector2f approx_hit_point_on_car(base_to_front, -base_to_side + 2 * base_to_side * p);
      float beta = angleAlongTurningPoint(approx_hit_point_on_car, turning_point_local);

      float alpha = collision_angle_total - beta;

      if (alpha < min_alpha) {
        min_alpha = alpha;
        min_alpha_point = point;
      }
    }

    // Side hit case
    bool side_collision = side_west_radius < point_radius && point_radius < side_east_radius;
    if (side_collision) {
      float collision_angle_total = angleAlongTurningPoint(point, turning_point_local);

      // TODO: Change from having anchor point in center of car
      // TODO: Remove big approvimation
      Eigen::Vector2f approx_hit_point_on_car(base_to_front, base_to_side);
      float beta = angleAlongTurningPoint(approx_hit_point_on_car, turning_point_local);

      float alpha = collision_angle_total - beta;

      if (alpha < min_alpha) {
        min_alpha = alpha;
        min_alpha_point = point;
      }
    }

    // #ifdef DEBUG_OD
    //       visualization::DrawArc(turning_point_local, side_west_radius * .5, -M_PI / 2,
    //                              -M_PI / 2 + collision_angle_total, 6, local_viz_msg_);
    // #endif
  }

  visualization::DrawArc(turning_point_local, side_west_radius, -M_PI / 2, -M_PI / 2 + min_alpha,
                         6, local_viz_msg_);

  path_option.alpha_collision = min_alpha;
  float closest_angle_to_target = std::min(min_alpha, max_angle_should_travel);
  path_option.closest_angle_to_target = closest_angle_to_target;
  path_option.closest_point =
      Eigen::Vector2f(r * cos(closest_angle_to_target), r * sin(closest_angle_to_target));
  path_option.obstruction = min_alpha_point;
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

  // TODO: radius scales with the inverse of curvature
  // when curvature is small, dc changes radius more
  const float dc = .1f;
  const float r_max = 1.1;
  const float c_max = 1.0f / r_max;
  const float c_start = 0.1;  // TODO: generalize for non-positive c
  const float c_end = c_max;

  int num_points = (c_end - c_start) / dc;

  static std::vector<PathOption> pathOptions(num_points, PathOption{});

  float c = c_start;

  float minDist = FP_INFINITE;
  int minDistIndex = 0;

  // TODO: transform point cloud first
  for (int i = 0; i < num_points; ++i, c += dc) {
    float r = 1 / c;
    pathOptions[i].curvature = c;
    this->maxDistanceTravelable(r, point_cloud_, pathOptions[i]);

    float distToPoint = (pathOptions[i].closest_point - this->nav_goal_disp_).norm();
    if (distToPoint < minDist) {
      minDist = distToPoint;
      minDistIndex = i;
    }
  }

  auto minDistPathOption = pathOptions[minDistIndex];
  auto minDistPathRadius = 1 / minDistPathOption.curvature;

  Eigen::Vector2f turning_point_local(0, minDistPathRadius);

  std::cout << "radius-    " << minDistPathRadius << "\n";

  visualization::DrawArc(turning_point_local, minDistPathRadius, -M_PI / 2,
                         -M_PI / 2 + minDistPathOption.closest_angle_to_target, 6, local_viz_msg_);

  // Update the remaining displacement.
  // Eigen::Vector2f inst_disp = Eigen::Rotation2Df(odom_angle_).toRotationMatrix() * odom_loc_ -
  //   Eigen::Rotation2Df(last_odom_pose_.angle).toRotationMatrix() * last_odom_pose_.translation;
  Eigen::Vector2f inst_disp = odom_loc_ - last_odom_pose_.translation;
  Eigen::Vector2f corrected_disp = Eigen::Rotation2Df(-last_odom_pose_.angle) * inst_disp;
  float inst_angular_disp = odom_angle_ - last_odom_pose_.angle;
  nav_goal_disp_ -= corrected_disp;
  nav_goal_disp_ = Eigen::Rotation2Df(-inst_angular_disp).toRotationMatrix() * nav_goal_disp_;

  // Transform predicted remaining displacement and point cloud
  // based on previous commands.
  Vector2f predicted_nav_goal_disp = nav_goal_disp_;
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

    for (auto& point : point_cloud_) {
      point -= disp_vec;
      point = rot * point;
    }
  }

  // TODO: use optimal curvature choice to to calculate remaining distance
  float remaining_angular_disp, remaining_distance;
  if (nav_curvature_ == 0) {
    remaining_angular_disp = 0;
    remaining_distance = predicted_nav_goal_disp.norm();
  } else {
    remaining_angular_disp = 2 * std::asin(nav_curvature_ * predicted_nav_goal_disp.norm() / 2);
    remaining_distance = remaining_angular_disp / nav_curvature_;
  }

  printf("[Navigation::Run]\n");
  printf("\todom loc: [%.2f, %.2f]\n", odom_loc_.x(), odom_loc_.y());
  printf("\theading: %.2fº\n", RadToDeg(odom_angle_));
  printf("\tinstantaneous displacement: [%.4f, %.4f]\n", inst_disp.x(), inst_disp.y());
  printf("\tinstantaneous displacement (corrected): [%.4f, %.4f]\n", corrected_disp.x(),
         corrected_disp.y());
  printf("\tinstantaneous angular difference: %.2fº\n", RadToDeg(inst_angular_disp));
  printf("\tcurvature: %.2f\n", nav_curvature_);
  printf("\tturning radius: %.2f\n", 1 / nav_curvature_);
  printf("\tremaining displacement: [%.2f, %.2f]\n", nav_goal_disp_.x(), nav_goal_disp_.y());
  printf("\tremaining angular displacement: %.2fº\n", RadToDeg(remaining_angular_disp));
  printf("\tremaining arc length: %.2f\n", remaining_distance);

  const float braking_distance = Sq(kMaxSpeed) / (2 * std::abs(kMaxDecel));
  // const float cur_speed = robot_vel_.norm();
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
  // TODO: use the optimal-path curvature
  drive_msg_.curvature = nav_curvature_;

  last_odom_pose_.Set(odom_angle_, odom_loc_);

  drive_msg_hist_.emplace_back(drive_msg_);
  if (drive_msg_hist_.size() > kControlHistorySize) {
    drive_msg_hist_.pop_front();
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
