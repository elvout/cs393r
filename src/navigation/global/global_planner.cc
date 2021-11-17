#include "navigation/global/global_planner.hh"

#include <future>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "navigation/global/map_graph.hh"
#include "navigation/global/path_finding.hh"
#include "vector_map/vector_map.h"

namespace {
constexpr float f32sNaN = std::numeric_limits<float>::signaling_NaN();
}

namespace navigation {
namespace global {

GlobalPlanner::GlobalPlanner(const std::string& map_file, int map_resolution)
    : nav_graph_(map_resolution, vector_map::VectorMap(map_file)),
      start_loc_(f32sNaN, f32sNaN),
      goal_loc_(f32sNaN, f32sNaN) {}

void GlobalPlanner::set_endpoints(const Eigen::Vector2f& new_start,
                                  const Eigen::Vector2f& new_goal) {
  if (new_start == start_loc_ && new_goal == goal_loc_) {
    return;
  }

  start_loc_ = new_start;
  goal_loc_ = new_goal;

  std::vector<Eigen::Vector2f> path = astar(nav_graph_, start_loc_, goal_loc_);

  // TODO: async with thread pool eventually
  std::promise<std::shared_ptr<std::vector<Eigen::Vector2f>>> temp_promise;
  temp_promise.set_value(std::make_shared<std::vector<Eigen::Vector2f>>(std::move(path)));

  // is this ordering of events always valid?
  path_ = temp_promise.get_future().share();
}

std::shared_ptr<std::vector<Eigen::Vector2f>> GlobalPlanner::get_plan_path() {
  if (!path_.valid()) {
    return std::shared_ptr<std::vector<Eigen::Vector2f>>(nullptr);
  } else {
    return path_.get();
  }
}

}  // namespace global
}  // namespace navigation
