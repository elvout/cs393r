#include "path_finding.hh"

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "map_graph.hh"
#include "util/matrix_hash.hh"

namespace {
constexpr double sqrt2 = 0x1.6a09e667f3bcdp+0;

// TODO: move this to the graph class?
using navigation::MapGraph;
double eight_conn_heuristic(const MapGraph::Vertex& v, const MapGraph::Vertex& goal) {
  const int dx = std::abs(goal.x() - v.x());
  const int dy = std::abs(goal.y() - v.y());

  return std::min(dx, dy) * sqrt2 + std::abs(dx - dy);
}

}  // namespace

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

std::vector<Eigen::Vector2f> astar(MapGraph& graph,
                                   const Eigen::Vector2f& start,
                                   const Eigen::Vector2f& goal) {
  using Vertex = MapGraph::Vertex;
  using Edge = MapGraph::Edge;

  const Vertex start_v = graph.coord_to_vertex(start);
  const Vertex goal_v = graph.coord_to_vertex(goal);

  std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> frontier;  // cost + heuristic
  std::unordered_map<Vertex, double, util::EigenMatrixHash<Vertex>> min_cost_to_v;  // cost only
  std::unordered_map<Vertex, Vertex, util::EigenMatrixHash<Vertex>> path_parent;

  min_cost_to_v[start_v] = 0;
  frontier.emplace(start_v, eight_conn_heuristic(start_v, goal_v));

  while (!frontier.empty()) {
    Edge min_edge = frontier.top();
    frontier.pop();

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
        frontier.emplace(neighbor, neighbor_cost + eight_conn_heuristic(neighbor, goal_v));
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
}  // namespace navigation
