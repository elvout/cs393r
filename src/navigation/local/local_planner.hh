#ifndef SRC_NAVIGATION_LOCAL_LOCAL_PLANNER_HH_
#define SRC_NAVIGATION_LOCAL_LOCAL_PLANNER_HH_

#include <deque>
#include <vector>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "shared/math/poses_2d.h"

namespace navigation {
namespace local {

class LocalPlanner {
 public:
  LocalPlanner();

  /**
   * Set the initial odometry pose. This should only be called once.
   */
  void init_odom_pose(const Eigen::Vector2f& odom_loc, const float odom_angle);

  /**
   * Set the target displacement in the robot's local reference frame.
   */
  void set_target_disp(const Eigen::Vector2f& new_target_disp);

  /**
   * Compute a drive message for a new time step.
   */
  amrl_msgs::AckermannCurvatureDriveMsg get_drive_msg(
      const Eigen::Vector2f& odom_loc,
      const float odom_angle,
      const std::vector<Eigen::Vector2f>& point_cloud,
      amrl_msgs::VisualizationMsg& local_viz_msg);

 private:
  Eigen::Vector2f target_disp_;  // in local frame
  pose_2d::Pose2Df last_odom_pose_;
  std::deque<amrl_msgs::AckermannCurvatureDriveMsg> drive_msg_hist_;
};

}  // namespace local
}  // namespace navigation

#endif  // SRC_NAVIGATION_LOCAL_LOCAL_PLANNER_HH_
