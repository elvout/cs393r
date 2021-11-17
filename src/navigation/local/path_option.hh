#ifndef SRC_NAVIGATION_LOCAL_PATH_OPTION_HH_
#define SRC_NAVIGATION_LOCAL_PATH_OPTION_HH_

#include <vector>
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"

namespace navigation {
namespace local {

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

}  // namespace local
}  // namespace navigation

#endif  // SRC_NAVIGATION_LOCAL_PATH_OPTION_HH_
