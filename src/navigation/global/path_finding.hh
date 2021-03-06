#ifndef SRC_NAVIGATION_GLOBAL_PATH_FINDING_HH_
#define SRC_NAVIGATION_GLOBAL_PATH_FINDING_HH_

#include "navigation/global/map_graph.hh"

#include <vector>
#include "eigen3/Eigen/Dense"

namespace navigation {
namespace global {

std::vector<Eigen::Vector2f> astar(MapGraph& graph,
                                   const Eigen::Vector2f& start,
                                   const Eigen::Vector2f& goal);

}  // namespace global
}  // namespace navigation

#endif  // SRC_NAVIGATION_GLOBAL_PATH_FINDING_HH_
