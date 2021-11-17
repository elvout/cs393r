#ifndef SRC_NAVIGATION_GLOBAL_GLOBAL_PLANNER_HH_
#define SRC_NAVIGATION_GLOBAL_GLOBAL_PLANNER_HH_

#include <future>
#include <memory>
#include <string>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "navigation/global/map_graph.hh"

namespace navigation {
namespace global {

class GlobalPlanner {
 public:
  GlobalPlanner(const std::string& map_file, int map_resolution);

  /**
   * Set the start and goal locations for the global plan.
   *
   * Replans if the start or goal locations have changed.
   */
  void set_endpoints(const Eigen::Vector2f& new_start, const Eigen::Vector2f& new_goal);

  /**
   * Return the global plan path, if it exists.
   *
   * The returned value may be nullptr or have a size of 0.
   */
  std::shared_ptr<std::vector<Eigen::Vector2f>> get_plan_path();

  // TODO: force a replan, e.g. after updating obstacles
  void replan();

  // TODO
  void update_obstacles(/* TODO: pointcloud type */);

 private:
  MapGraph nav_graph_;

  Eigen::Vector2f start_loc_;
  Eigen::Vector2f goal_loc_;
  std::shared_future<std::shared_ptr<std::vector<Eigen::Vector2f>>> path_;
};

}  // namespace global
}  // namespace navigation

#endif  // SRC_NAVIGATION_GLOBAL_GLOBAL_PLANNER_HH_
