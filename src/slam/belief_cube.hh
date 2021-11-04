#ifndef SRC_SLAM_BELIEF_CUBE_HH_
#define SRC_SLAM_BELIEF_CUBE_HH_

#include <unordered_map>
#include "eigen3/Eigen/Dense"
#include "util/matrix_hash.hh"

/**
 * TODO: doc
 *
 * Implemented as a dictionary of keys.
 * Although this matrix is not necessarily sparse, it just makes it
 * easier to index negative values.
 *  - TODO: use a threshold to sparsify the matrix?
 *
 * Use degrees [0, 359] - want integer values for (consistent?) hashing
 *  1 degree is pretty good resolution
 *  remember to constrain angles to this range
 */
class BeliefCube {
  using Point = Eigen::Vector3i;  // [x; y; z=theta]

 private:
  static constexpr int tx_resolution_ = 4;   // centimeters
  static constexpr int rot_resolution_ = 2;  // degrees

 private:
  std::unordered_map<Point, double, util::EigenMatrixHash<Point>> cube_;
};

#endif  // SRC_SLAM_BELIEF_CUBE_HH_
