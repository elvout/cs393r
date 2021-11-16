#ifndef SRC_NAVIGATION_NAVIGATION_GRAPH_HH_
#define SRC_NAVIGATION_NAVIGATION_GRAPH_HH_

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "util/matrix_hash.hh"
#include "vector_map/vector_map.h"

namespace navigation {

/**
 * A discretized graph representation of a VectorMap for path planning
 * via graph search.
 *
 * The graph is weighted, undirected, and 8-connected.
 *
 * The graph is represented using a mapped adjacency list since:
 *  - We're mainly concerned with iterating over all adjacent vertices
 *    rather than testing for adjacency.
 *  - Testing for adjacency runs in constant time for an 8-connected
 *    graph.
 *
 * The graph is lazily generated as it is queried since the entire map
 * generally won't be explored.
 *
 * Vertex and edge weight values are represented in centimeters.
 */
class MapGraph {
 public:  // Data types
  using Vertex = Eigen::Vector2i;

  struct Edge {
    Edge(const Vertex& v, double w);
    Edge(Vertex&& v, double w);

    bool operator<(const Edge& other) const;
    bool operator>(const Edge& other) const;

    Vertex dest;
    double weight;
  };

  using MappedAdjList =
      std::unordered_map<Vertex, std::vector<Edge>, util::EigenMatrixHash<Vertex>>;

 public:  // Public API
  /**
   * Construct an empty graph with the specified resolution and a vertex
   * exclusion list using the vector map.
   */
  MapGraph(const unsigned int resolution, const vector_map::VectorMap& map);

  /**
   * Popluate the edges of the specified vertex and return an iterator
   * to the internal map entry.
   */
  MappedAdjList::iterator add_vertex(const Vertex& v);

  /**
   * Populate the edges of the vertex corresponding to the specified
   * coordinate and return the corresponding vertex.
   */
  const Vertex add_vertex(const Eigen::Vector2f& coord);

  /**
   * Return the edges adjacent to the specified vertex.
   */
  const std::vector<Edge>& neighbors(const Vertex& v);

  /**
   * Return whether the specified vertex is in the obstacle set.
   */
  bool is_obstacle(const Vertex& v) const;

  /**
   * Discretize a "real"-valued coordinate using the graph resolution.
   */
  Vertex coord_to_vertex(const Eigen::Vector2f& coord) const;

  /**
   * Convert a discretized graph vertex to a "real"-valued coordinate.
   * Returns the coordinate corresponding to the center of the
   * discretized cell.
   */
  Eigen::Vector2f vertex_to_coord(const Vertex& v) const;

 private:
  /**
   * Round a "real"-valued measurement in meters to the nearest
   * discretized bin and return the center of the bin in centimeters.
   */
  int meters_to_index(const double meters) const;

  /**
   * Convert a bin value in centimeters to a "real"-valued measurement
   * in meters.
   */
  double index_to_meters(const int index) const;

  const unsigned int resolution_;  // centimeters
  std::unordered_set<Vertex, util::EigenMatrixHash<Vertex>> obstacles_;
  MappedAdjList adjlist_;
};

}  // namespace navigation

#endif  // SRC_NAVIGATION_NAVIGATION_GRAPH_HH_
