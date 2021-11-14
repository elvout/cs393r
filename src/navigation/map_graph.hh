#ifndef SRC_NAVIGATION_NAVIGATION_GRAPH_HH_
#define SRC_NAVIGATION_NAVIGATION_GRAPH_HH_

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "util/matrix_hash.hh"
#include "vector_map/vector_map.h"

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

    Vertex dest;
    double weight;
  };

  using MappedAdjList =
      std::unordered_map<Vertex, std::vector<Edge>, util::EigenMatrixHash<Vertex>>;

 public:  // Public API
  MapGraph(const unsigned int resolution, const vector_map::VectorMap& map);

  MappedAdjList::iterator add_vertex(const Vertex& v);
  const Vertex add_vertex(const Eigen::Vector2f& coord);
  const std::vector<Edge>& neighbors(const Vertex& v);

  Vertex coord_to_vertex(const Eigen::Vector2f& coord) const;
  Eigen::Vector2f vertex_to_coord(const Vertex& v) const;

 private:
  int meters_to_index(const double meters) const;
  double index_to_meters(const int index) const;

  const unsigned int resolution_;  // centimeters
  std::unordered_set<Vertex, util::EigenMatrixHash<Vertex>> obstacles_;
  MappedAdjList adjlist_;
};

#endif  // SRC_NAVIGATION_NAVIGATION_GRAPH_HH_
