#include "navigation/local/local_planner.hh"

#include <cmath>
#include <vector>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "navigation/constants.hh"
#include "navigation/local/path_option.hh"
#include "shared/math/math_util.h"
#include "visualization/visualization.h"

namespace {

using navigation::local::PathOption;

/**
 * Return the best path to the target to take based on point cloud data.
 *
 * The best path is defined as the path with minimal distace of closest
 * approach (if completely disjoint or externally tangent) or periapsis
 * (if intersecting) to the target.
 *
 * TODO: factor in clearance
 */
PathOption findBestPath(const std::vector<Eigen::Vector2f>& point_cloud,
                        const Eigen::Vector2f& target,
                        amrl_msgs::VisualizationMsg& local_viz_msg) {
  using navigation::constants::kMaxSteeringAngle;
  using navigation::constants::kMinSteeringAngle;
  using navigation::constants::kWheelBase;

  constexpr size_t kNumSteps = 50;
  const float angle_step_size = (kMaxSteeringAngle - kMinSteeringAngle) / kNumSteps;

  std::vector<PathOption> path_options;
  path_options.reserve(kNumSteps);

  for (size_t i = 0; i < kNumSteps; i++) {
    float steering_angle = kMinSteeringAngle + angle_step_size * i;
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

    path.visualize(local_viz_msg, 0xdddddd);
  }

  // visualize the best path in green
  closest_approach_path.visualize(local_viz_msg, 0x00ff00);
  return closest_approach_path;
}
}  // namespace

namespace navigation {
namespace local {

LocalPlanner::LocalPlanner() : target_disp_(), last_odom_pose_(), drive_msg_hist_() {}

void LocalPlanner::init_odom_pose(const Eigen::Vector2f& odom_loc, const float odom_angle) {
  last_odom_pose_.Set(odom_angle, odom_loc);
}

void LocalPlanner::set_target_disp(const Eigen::Vector2f& new_target_disp) {
  target_disp_ = new_target_disp;
  // TODO: what to do with drive_msg_hist_?
}

amrl_msgs::AckermannCurvatureDriveMsg LocalPlanner::get_drive_msg(
    const Eigen::Vector2f& odom_loc,
    const float odom_angle,
    const std::vector<Eigen::Vector2f>& point_cloud,
    amrl_msgs::VisualizationMsg& local_viz_msg) {
  using constants::kBrakingDistance;
  using constants::kMaxAccel;
  using constants::kMaxDecel;
  using constants::kMaxSpeed;
  using constants::kUpdateFrequency;

  // Update the displacement target based on new odometry data.
  const Eigen::Vector2f odom_disp = odom_loc - last_odom_pose_.translation;
  const Eigen::Vector2f local_disp = Eigen::Rotation2Df(-last_odom_pose_.angle) * odom_disp;
  const float local_angular_disp = odom_angle - last_odom_pose_.angle;
  target_disp_ -= local_disp;
  target_disp_ = Eigen::Rotation2Df(-local_angular_disp) * target_disp_;
  last_odom_pose_.Set(odom_angle, odom_loc);

  // TODO: better documentation of forward prediction

  // Transform predicted remaining displacement and point cloud
  // based on previous commands.
  Eigen::Vector2f predicted_target_disp = target_disp_;
  std::vector<Eigen::Vector2f> predicted_point_cloud = point_cloud;
  for (const auto& msg : drive_msg_hist_) {
    const float arc_len = msg.velocity / kUpdateFrequency;
    const float turning_radius = 1 / msg.curvature;
    const float subtended_angle = arc_len / turning_radius;
    const Eigen::Rotation2Df rot(-subtended_angle);

    Eigen::Vector2f disp_vec(0, 0);
    if (msg.curvature == 0) {
      disp_vec.x() = arc_len;
    } else {
      disp_vec.x() = turning_radius * std::sin(subtended_angle);
      disp_vec.y() = turning_radius - turning_radius * std::cos(subtended_angle);
    }

    predicted_target_disp -= disp_vec;
    predicted_target_disp = rot * predicted_target_disp;

    for (auto& point : predicted_point_cloud) {
      point -= disp_vec;
      point = rot * point;
    }
  }

  // Debugging: draw predicted point cloud
  // The point cloud will generally be shifted a little towards the car's
  // location. Honestly this isn't super useful unless something is very
  // wrong.
  // for (auto& point : predicted_point_cloud) {
  //   visualization::DrawPoint(point, 0xd1e231, local_viz_msg);
  // }

  const PathOption best_path =
      findBestPath(predicted_point_cloud, predicted_target_disp, local_viz_msg);
  const float remaining_distance = best_path.free_path_length;
  float cur_speed;  // technically the speed of last predicted msg
  if (drive_msg_hist_.empty()) {
    /* NOTE: In assignment 1, we used
     *   cur_speed = robot_vel_.norm();
     *
     * robot_vel_ is part of the Navigation class; written to by ::UpdateOdometry
     *
     * However, we will assume initialization happens while the
     * car is at rest. (I also don't want to pass more parameters
     * than necessary).
     */
    cur_speed = 0;
  } else {
    cur_speed = drive_msg_hist_.back().velocity;
  }

  float new_velocity = 0;
  if (remaining_distance <= kBrakingDistance) {
    new_velocity = std::max(0.0f, cur_speed + kMaxDecel / kUpdateFrequency);
  } else {
    new_velocity = std::min(kMaxSpeed, cur_speed + kMaxAccel / kUpdateFrequency);
  }

  drive_msg_hist_.emplace_back();  // autogen only made the default constructor >:(
  drive_msg_hist_.back().velocity = new_velocity;
  drive_msg_hist_.back().curvature = best_path.curvature;
  if (drive_msg_hist_.size() > constants::kControlHistorySize) {
    drive_msg_hist_.pop_front();
  }

  // printf("[LocalPlanner::get_drive_msg]\n");
  // printf("\tinstantaneous displacement (odom): [%.4f, %.4f]\n", odom_disp.x(), odom_disp.y());
  // printf("\tinstantaneous displacement (local frame): [%.4f, %.4f]\n", local_disp.x(),
  //        local_disp.y());
  // printf("\tinstantaneous angular difference: %.2fº\n",
  // math_util::RadToDeg(local_angular_disp)); printf("\tcommand curvature: %.2f\n",
  // best_path.curvature); printf("\tcommand turning radius: %.2f\n", 1 / best_path.curvature);
  // printf("\tcommand speed: %.2f\n", new_velocity);
  // printf("\tremaining displacement: [%.2f, %.2f]\n", predicted_target_disp.x(),
  //        predicted_target_disp.y());
  // printf("\tremaining arc length: %.2f\n", remaining_distance);

  visualization::DrawCross(target_disp_, 0.25, 0xff0000, local_viz_msg);

  return drive_msg_hist_.back();
}

}  // namespace local
}  // namespace navigation
