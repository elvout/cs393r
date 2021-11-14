#include "map_graph.hh"

#include <array>
#include <cmath>
#include <stdexcept>
#include "math/line2d.h"
#include "math/math_util.h"
#include "vector_map/vector_map.h"

namespace {
constexpr double sqrt2 = 0x1.6a09e667f3bcdp+0;

const std::array<MapGraph::Edge, 8> expansion_dirs{{
    {MapGraph::Vertex(1, 0), 1.0},
    {MapGraph::Vertex(0, 1), 1.0},
    {MapGraph::Vertex(-1, 0), 1.0},
    {MapGraph::Vertex(0, -1), 1.0},
    {MapGraph::Vertex(1, 1), sqrt2},
    {MapGraph::Vertex(1, -1), sqrt2},
    {MapGraph::Vertex(-1, 1), sqrt2},
    {MapGraph::Vertex(-1, -1), sqrt2},
}};
}  // namespace

MapGraph::Edge::Edge(const MapGraph::Vertex& v, double w) : dest(v), weight(w) {}
MapGraph::Edge::Edge(MapGraph::Vertex&& v, double w) : dest(v), weight(w) {}

MapGraph::MapGraph(const unsigned int resolution, const vector_map::VectorMap& map)
    : resolution_(resolution), obstacles_(), adjlist_() {
  if (resolution_ == 0) {
    throw std::runtime_error("MapGraph: resolution cannot be zero.");
  }

  const double res_meters = resolution / 100.0;
  for (const geometry::line2f& map_line : map.lines) {
    const Eigen::Vector2f& p0 = map_line.p0;
    const Eigen::Vector2f& p1 = map_line.p1;

    const float dist_x = (p1.x() - p0.x());
    const float dist_y = (p1.y() - p0.y());
    const float dist = (p1 - p0).norm();

    const unsigned int n_steps = static_cast<unsigned int>(std::ceil(dist / res_meters));
    const Eigen::Vector2f step_size(dist_x / n_steps, dist_y / n_steps);

    Eigen::Vector2f coord = p0;
    for (unsigned int step_i = 0; step_i < n_steps; step_i++) {
      obstacles_.insert(coord_to_vertex(coord));
      coord += step_size;
    }
  }
}

MapGraph::MappedAdjList::iterator MapGraph::add_vertex(const Vertex& v) {
  auto [it, inserted] = adjlist_.emplace(v, std::vector<Edge>{});

  if (inserted) {
    std::vector<Edge>& edges = it->second;
    edges.reserve(expansion_dirs.size());

    for (const Edge& edge_delta : expansion_dirs) {
      const Vertex dest = v + edge_delta.dest * resolution_;
      if (obstacles_.count(dest) == 0) {
        edges.emplace_back(std::move(dest), edge_delta.weight * resolution_);
      }
    }
  }

  return it;
}

const MapGraph::Vertex MapGraph::add_vertex(const Eigen::Vector2f& coord) {
  const Vertex v = coord_to_vertex(coord);
  add_vertex(v);
  return v;
}

const std::vector<MapGraph::Edge>& MapGraph::neighbors(const Vertex& v) {
  auto it = adjlist_.find(v);
  if (it == adjlist_.cend()) {
    it = add_vertex(v);
  }

  return it->second;
}

int MapGraph::meters_to_index(const double meters) const {
  // symmetric rounding towards 0
  const double unsigned_cm = std::abs(meters) * 100.0;
  const double bin_center = (unsigned_cm + resolution_ * 0.5) / resolution_;
  return static_cast<int>(bin_center) * resolution_ * math_util::Sign(meters);
}

double MapGraph::index_to_meters(const int index) const {
  return index / 100.0;
}

MapGraph::Vertex MapGraph::coord_to_vertex(const Eigen::Vector2f& coord) const {
  return Vertex(meters_to_index(coord.x()), meters_to_index(coord.y()));
}

Eigen::Vector2f MapGraph::vertex_to_coord(const Vertex& v) const {
  return Eigen::Vector2f(index_to_meters(v.x()), index_to_meters(v.y()));
}
