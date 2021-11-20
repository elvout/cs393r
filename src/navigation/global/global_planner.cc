#include "navigation/global/global_planner.hh"

#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "navigation/global/map_graph.hh"
#include "navigation/global/path_finding.hh"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "vector_map/vector_map.h"

namespace {
constexpr float f32sNaN = std::numeric_limits<float>::signaling_NaN();
}

namespace navigation {
namespace global {

GlobalPlanner::GlobalPlanner(const std::string& map_file, int map_resolution)
    : vec_map_(map_file),
      nav_graph_(map_resolution, vec_map_),
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

std::optional<Eigen::Vector2f> GlobalPlanner::intermediate_waypoint(const Eigen::Vector2f& loc,
                                                                    const float angle) {
  std::shared_ptr<std::vector<Eigen::Vector2f>> plan_path_p = get_plan_path();
  if (!plan_path_p || plan_path_p->empty()) {
    return std::make_optional<Eigen::Vector2f>();
  }

  // Find the furthest (in terms of index in the plan) vertex with a straight-line
  // sight from the location and project it into the local reference frame.
  //
  // TODO: find the maximally furthest point along the plan path
  constexpr double max_waypoint_distance = 3.0;
  const std::vector<Eigen::Vector2f>& plan_path = *plan_path_p;
  auto furthest_vertex = plan_path.cend();

  // TODO: is there a more efficient way of doing this (like bsearch?)
  for (auto it = plan_path.begin(); it != plan_path.cend(); it++) {
    const geometry::line2f line_of_sight(loc, *it);

    if ((loc - *it).norm() > max_waypoint_distance) {
      continue;
    }

    bool intersection = false;
    for (const geometry::line2f& map_line : vec_map_.lines) {
      if (line_of_sight.Intersects(map_line)) {
        intersection = true;
        break;
      }
    }

    if (!intersection) {
      furthest_vertex = it;
    }
  }

  if (furthest_vertex == plan_path.cend()) {
    return std::make_optional<Eigen::Vector2f>();
  }

  const Eigen::Rotation2Df global_to_loc_rot(-angle);
  const Eigen::Vector2f local_target = global_to_loc_rot * (*furthest_vertex - loc);
  return std::make_optional<Eigen::Vector2f>(std::move(local_target));
}

}  // namespace global
}  // namespace navigation
