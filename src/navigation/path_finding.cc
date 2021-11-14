#include "path_finding.hh"

#include <queue>
#include <unordered_map>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "map_graph.hh"
#include "util/matrix_hash.hh"

namespace navigation {

/**
 * Dijkstra's algorithm implemented with a binary heap and lazy relaxation.
 */
std::vector<Eigen::Vector2f> dijkstra(MapGraph& graph,
                                      const Eigen::Vector2f& start,
                                      const Eigen::Vector2f& goal) {
  using Vertex = MapGraph::Vertex;
  using Edge = MapGraph::Edge;

  const Vertex start_v = graph.coord_to_vertex(start);
  const Vertex goal_v = graph.coord_to_vertex(goal);

  std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> frontier;
  std::unordered_map<Vertex, double, util::EigenMatrixHash<Vertex>> min_cost_to_v;
  std::unordered_map<Vertex, Vertex, util::EigenMatrixHash<Vertex>> path_parent;

  min_cost_to_v[start_v] = 0;
  frontier.emplace(start_v, 0);

  while (!frontier.empty()) {
    Edge min_edge = frontier.top();
    frontier.pop();

    // lazy relaxation, the min cost to this vertex has already been computed
    if (min_cost_to_v[min_edge.dest] < min_edge.weight) {
      continue;
    }

    if (min_edge.dest == goal_v) {
      break;
    }

    for (const Edge& e : graph.neighbors(min_edge.dest)) {
      const Vertex& neighbor = e.dest;
      const double neighbor_cost = min_edge.weight + e.weight;

      auto cost_lookup_it = min_cost_to_v.find(neighbor);
      if (cost_lookup_it == min_cost_to_v.cend() || neighbor_cost < cost_lookup_it->second) {
        min_cost_to_v[neighbor] = neighbor_cost;
        path_parent[neighbor] = min_edge.dest;
        frontier.emplace(neighbor, neighbor_cost);
      }
    }
  }

  if (min_cost_to_v.find(goal_v) == min_cost_to_v.cend()) {
    std::cout << "path to goal not found" << std::endl;
  } else {
    std::cout << "cost to goal: " << min_cost_to_v[goal_v] << std::endl;
  }

  std::vector<Eigen::Vector2f> path;

  return path;
}
}  // namespace navigation
