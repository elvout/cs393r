#include "map_graph.hh"

#include <cmath>
#include <stdexcept>
#include "math/line2d.h"
#include "vector_map/vector_map.h"

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

int MapGraph::meters_to_index(const double meters) const {
  const double cm = meters * 100.0;
  const double bin_center = (cm + resolution_ * 0.5) / resolution_;
  return static_cast<int>(bin_center);
}

MapGraph::Vertex MapGraph::coord_to_vertex(const Eigen::Vector2f& coord) const {
  return Vertex(meters_to_index(coord.x()), meters_to_index(coord.y()));
}
