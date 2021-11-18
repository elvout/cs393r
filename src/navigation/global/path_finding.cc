#include "navigation/global/path_finding.hh"

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "map_graph.hh"
#include "util/matrix_hash.hh"

namespace navigation {
namespace global {

std::vector<Eigen::Vector2f> astar(MapGraph& graph,
                                   const Eigen::Vector2f& start,
                                   const Eigen::Vector2f& goal) {
  using Vertex = MapGraph::Vertex;
  using Edge = MapGraph::Edge;

  const Vertex start_v = graph.coord_to_vertex(start);
  const Vertex goal_v = graph.coord_to_vertex(goal);

  if (graph.is_obstacle(goal_v)) {
    std::cout << "goal is obstacle" << std::endl;
    return std::vector<Eigen::Vector2f>();
  }

  std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> frontier;  // cost + heuristic

  // TODO: these can probably all be consolidated into one map
  std::unordered_map<Vertex, double, util::EigenMatrixHash<Vertex>> min_cost_to_v;  // cost only
  std::unordered_map<Vertex, Vertex, util::EigenMatrixHash<Vertex>> path_parent;
  std::unordered_set<Vertex, util::EigenMatrixHash<Vertex>> opt_visited;

  min_cost_to_v[start_v] = 0;
  frontier.emplace(start_v, graph.heuristic(start_v, goal_v));

  while (!frontier.empty()) {
    Edge min_edge = frontier.top();
    frontier.pop();

    // This reduces the number of vertices expanded when no path
    // exists between the start and goal. (TODO: why?)
    if (opt_visited.count(min_edge.dest) == 1) {
      continue;
    }
    opt_visited.insert(min_edge.dest);

    if (min_edge.dest == goal_v) {
      break;
    }

    for (const Edge& e : graph.neighbors(min_edge.dest)) {
      const Vertex& neighbor = e.dest;
      const double neighbor_cost = min_cost_to_v[min_edge.dest] + e.weight;

      auto cost_lookup_it = min_cost_to_v.find(neighbor);
      if (cost_lookup_it == min_cost_to_v.cend() || neighbor_cost < cost_lookup_it->second) {
        min_cost_to_v[neighbor] = neighbor_cost;
        path_parent[neighbor] = min_edge.dest;
        frontier.emplace(neighbor, neighbor_cost + graph.heuristic(neighbor, goal_v));
      }
    }
  }

  std::vector<Eigen::Vector2f> path;

  if (min_cost_to_v.find(goal_v) == min_cost_to_v.cend()) {
    std::cout << "path to goal not found" << std::endl;
  } else {
    std::cout << "cost to goal: " << min_cost_to_v[goal_v] << std::endl;

    Vertex& current_v = const_cast<Vertex&>(goal_v);
    while (current_v != start_v) {
      path.push_back(graph.vertex_to_coord(current_v));
      current_v = path_parent[current_v];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
  }

  return path;
}

}  // namespace global
}  // namespace navigation
