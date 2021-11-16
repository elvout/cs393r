#ifndef SRC_NAVIGATION_PATH_FINDING_HH_
#define SRC_NAVIGATION_PATH_FINDING_HH_

#include "map_graph.hh"

#include <vector>
#include "eigen3/Eigen/Dense"

namespace navigation {

std::vector<Eigen::Vector2f> dijkstra(MapGraph& graph,
                                      const Eigen::Vector2f& start,
                                      const Eigen::Vector2f& goal);

std::vector<Eigen::Vector2f> astar(MapGraph& graph,
                                   const Eigen::Vector2f& start,
                                   const Eigen::Vector2f& goal);

}  // namespace navigation

#endif  // SRC_NAVIGATION_PATH_FINDING_HH_
