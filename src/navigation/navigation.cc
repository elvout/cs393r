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

/**
 * Return an angle specfied in radians constrained to the range [0, 2PI).
 *
 * TODO: move this to the shared math library?
 */
template <typename T>
T constrainAngle(T angle) {
  static_assert(std::is_floating_point<T>::value, "");

  angle = fmod(angle, M_2PI);
  // angle can still be negative, but its absolute value will be < 2PI
  return fmod(angle + M_2PI, M_2PI);
}

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

/**
 * Returns the angular distance in radians between the base link of the
 * car and an arbitrary point as viewed from the center of turning.
 *
 * `point` is a coordinate in the base link reference frame.
 *
 * `radius` is the turning radius of the car. A positive value indicates
 * a left turn and a negative value indicates a right turn.
 */
float angularDistanceToPoint(Eigen::Vector2f point, const float radius) {
  Vector2f center_to_base(0, -radius);

  // Transform vectors into the reference frame of the center of turning,
  // in which the vector from the center of turning to the base link is
  // the x-axis.
  Eigen::Rotation2Df rot(std::asin(Sign(radius)));
  point = rot * point;
  center_to_base = rot * center_to_base;

  assert(center_to_base.x() >= 0);
  assert(std::abs(center_to_base.y()) < kEpsilon);

  float angular_dist = std::atan2(point.y(), point.x());
  angular_dist = constrainAngle(angular_dist);

  // `angular_dist` describes a counterclockwise angle from the x-axis in the
  // center-of-turning reference frame. In the case of a right turn, return
  // the clockwise angle.
  if (radius >= 0) {
    return angular_dist;
  } else {
    return M_2PI - angular_dist;
  }
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

  // TODO: Non-uniform car width

  // Using cardinal directions in car reference frame
  Eigen::Vector2f ne_vector(base_to_front, base_to_side);
  Eigen::Vector2f se_vector(base_to_front, -base_to_side);
  Eigen::Vector2f nw_vector(-base_to_front, base_to_side);
  Eigen::Vector2f sw_vector(-base_to_front, -base_to_side);

#ifdef DEBUG_OD
  visualization::DrawLine(side_west_vector_local, side_east_vector_local, 5, local_viz_msg_);
  visualization::DrawLine(front_south_vector_local, front_north_vector_local, 5, local_viz_msg_);
#endif

  float ne_radius = (turning_point_local - ne_vector).norm();
  float se_radius = (turning_point_local - se_vector).norm();
  float nw_radius = (turning_point_local - nw_vector).norm();
  float sw_radius = (turning_point_local - sw_vector).norm();

#ifdef DEBUG_OD
  visualization::DrawArc(turning_point_local, front_north_radius, 0, 2 * M_PI, 1, local_viz_msg_);
  visualization::DrawArc(turning_point_local, front_south_radius, 0, 2 * M_PI, 1, local_viz_msg_);
  visualization::DrawArc(turning_point_local, side_west_radius, 0, 2 * M_PI, 1, local_viz_msg_);
  visualization::DrawArc(turning_point_local, side_east_radius, 0, 2 * M_PI, 1, local_viz_msg_);
#endif

  float max_angle_should_travel = angularDistanceToPoint(this->nav_goal_disp_, r);

  float min_alpha = 2 * M_PI;
  // TODO: What to do with this when there's no collision?
  Eigen::Vector2f min_alpha_point;

  for (auto point : point_cloud_) {
    // visualization::DrawPoint(point, 4, local_viz_msg_);

    float point_radius = (turning_point_local - point).norm();

    // Front hit case
    // r0 < r1
    float r0_front = r > 0 ? ne_radius : se_radius;
    float r1_front = r > 0 ? se_radius : ne_radius;
    bool front_collision = r0_front < point_radius && point_radius < r1_front;

    if (front_collision) {
      float collision_angle_total = angularDistanceToPoint(point, r);

      // From brainstorm scan
      float d = (turning_point_local - point).norm();
      float p = (d - r0_front) / (r1_front - r0_front);

      Eigen::Vector2f approx_hit_point_on_car(base_to_front, -base_to_side + 2 * base_to_side * p);
      float beta = angularDistanceToPoint(approx_hit_point_on_car, r);

      float alpha = collision_angle_total - beta;
      assert(alpha > 0);

      if (alpha < min_alpha) {
        min_alpha = alpha;
        min_alpha_point = point;
      }
    }

    // Side hit case
    float r0_side = r > 0 ? nw_radius : sw_radius;
    float r1_side = r > 0 ? ne_radius : se_radius;
    bool side_collision = r0_side < point_radius && point_radius < r1_side;
    if (side_collision) {
      float collision_angle_total = angularDistanceToPoint(point, r);

      // TODO: Potentially problematic
      Eigen::Vector2f approx_hit_point_on_car(0, base_to_side * Sign(r));
      float beta = angularDistanceToPoint(approx_hit_point_on_car, r);

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

  assert(min_alpha > 0);

  float anchor_radius = (turning_point_local).norm();
  if (r > 0)
    visualization::DrawArc(turning_point_local, anchor_radius, -M_PI / 2, -M_PI / 2 + min_alpha, 6,
                           local_viz_msg_);
  else
    visualization::DrawArc(turning_point_local, anchor_radius, M_PI / 2 - min_alpha, M_PI / 2, 6,
                           local_viz_msg_);

  path_option.alpha_collision = min_alpha;
  float closest_angle_to_target = std::min(min_alpha, max_angle_should_travel);
  path_option.closest_angle_to_target = closest_angle_to_target;
  path_option.closest_point = Eigen::Vector2f(std::abs(r) * sin(closest_angle_to_target),
                                              r - r * cos(closest_angle_to_target));

  // draw the target point
  visualization::DrawCross(path_option.closest_point, 0.03, 3, local_viz_msg_);
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

  constexpr float kWheelBase = 0.33;  // m
  const float min_steering_angle = atan(kWheelBase / -1.1);
  const float max_steering_angle = atan(kWheelBase / 1.1);
  const size_t kNumSteps = 50;
  const float angle_step_size = (max_steering_angle - min_steering_angle) / kNumSteps;

  std::vector<PathOption> pathOptions(kNumSteps);
  float minDist = std::numeric_limits<float>::max();
  auto minDistPathOption = pathOptions.begin();

  for (size_t i = 0; i < kNumSteps; i++) {
    auto cur_option = pathOptions.begin() + i;

    float steering_angle = min_steering_angle + angle_step_size * i;
    float r = kWheelBase / tan(steering_angle);

    cur_option->curvature = 1 / r;
    maxDistanceTravelable(r, point_cloud_, *cur_option);

    // TODO: use predicted nav goal displacement
    float distToPoint = (cur_option->closest_point - nav_goal_disp_).norm();
    if (distToPoint < minDist) {
      minDist = distToPoint;
      minDistPathOption = cur_option;
    }
  }

  auto maxDistPathRadius = 1 / minDistPathOption->curvature;
  auto closest_angle_to_target = minDistPathOption->closest_angle_to_target;

  Eigen::Vector2f turning_point_local(0, maxDistPathRadius);

  if (maxDistPathRadius > 0)
    visualization::DrawArc(turning_point_local, maxDistPathRadius, -M_PI / 2,
                           -M_PI / 2 + closest_angle_to_target, 6, local_viz_msg_);
  else
    visualization::DrawArc(turning_point_local, maxDistPathRadius,
                           M_PI / 2 - closest_angle_to_target, M_PI / 2, 6, local_viz_msg_);

  // Update the remaining displacement based on odometry data.
  const Eigen::Vector2f odom_disp = odom_loc_ - last_odom_pose_.translation;
  const Eigen::Vector2f reference_disp = Eigen::Rotation2Df(-last_odom_pose_.angle) * odom_disp;
  const float inst_angular_disp = odom_angle_ - last_odom_pose_.angle;
  nav_goal_disp_ -= reference_disp;
  nav_goal_disp_ = Eigen::Rotation2Df(-inst_angular_disp) * nav_goal_disp_;

  float remaining_distance =
      minDistPathOption->alpha_collision / std::abs(minDistPathOption->curvature);

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
  drive_msg_.curvature = minDistPathOption->curvature;

  last_odom_pose_.Set(odom_angle_, odom_loc_);

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
