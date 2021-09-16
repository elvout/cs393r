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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <deque>
#include <vector>

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "shared/math/poses_2d.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
class NodeHandle;
}  // namespace ros

namespace navigation {

constexpr float kUpdateFrequency = 20.0f;  // 1 / s

// dynamic constraints
constexpr float kMaxSpeed = 1.0f;   // m / s
constexpr float kMaxAccel = 4.0f;   // m / s^2
constexpr float kMaxDecel = -4.0f;  // m / s^2

constexpr float kActuationLatency = 0.2f;  // s
constexpr float kControlHistorySize = kActuationLatency * kUpdateFrequency;

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float free_path_subtended_angle;
  Eigen::Vector2f closest_point_to_target;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PathOption() = default;
  PathOption(const float curvature,
             const std::vector<Eigen::Vector2f>& point_cloud,
             const Eigen::Vector2f& target);

  /**
   * Visualize this path with an arc of length `free_path_length`.
   *
   * `msg` should be the `VisualizationMsg` for the robot's local reference frame.
   *
   * `color` is a hex color code for the path.
   */
  void visualize(amrl_msgs::VisualizationMsg& msg, uint32_t color) const;
};

class Navigation {
 public:
  // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud, double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  /**
   * Set the remaining displacement for navigation.
   */
  void SetNavDisplacement(const float dx, const float dy);

  inline bool odom_initialized() const { return odom_initialized_; }

 private:
  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Navigation goal remaining displacement.
  Eigen::Vector2f nav_goal_disp_;
  // Odometry pose from the last Run() invocation.
  pose_2d::Pose2Df last_odom_pose_;
  // Drive Message History
  std::deque<amrl_msgs::AckermannCurvatureDriveMsg> drive_msg_hist_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
