#ifndef SRC_SLAM_BELIEF_CUBE_HH_
#define SRC_SLAM_BELIEF_CUBE_HH_

#include <unordered_map>
#include "eigen3/Eigen/Dense"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"
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
  using Point = Eigen::Vector3i;  // [x=dx; y=dy; z=dtheta]

 public:
  BeliefCube();

  void eval(const RasterMap& ref_map,
            const Eigen::Vector2f& ref_loc,
            const double ref_angle,
            const Eigen::Vector2f& odom_disp,
            const double odom_angle_disp,
            const sensor_msgs::LaserScan& new_obs);

 private:
  static constexpr int tx_resolution_ = 4;     // centimeters
  static constexpr int rot_resolution_ = 2;    // degrees
  static constexpr int tx_windowsize_ = 100;   // centimeters, inclusive
  static constexpr int rot_windowsize_ = 359;  // degrees, inclusive

  Point binify(const double x, const double y, const double rad) const;
  Point binify(const Eigen::Vector2f& coord, const double rad) const;

 private:
  std::unordered_map<Point, double, util::EigenMatrixHash<Point>> cube_;
};

#endif  // SRC_SLAM_BELIEF_CUBE_HH_
