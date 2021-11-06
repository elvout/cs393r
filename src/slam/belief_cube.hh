#ifndef SRC_SLAM_BELIEF_CUBE_HH_
#define SRC_SLAM_BELIEF_CUBE_HH_

#include <optional>
#include <unordered_map>
#include <utility>
#include "eigen3/Eigen/Dense"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"
#include "util/matrix_hash.hh"

namespace slam {

/**
 * A discretized representation of the belief (in delta space) at an
 * arbitrary time step.
 *
 * Implemented as a dictionary of keys. Although the lookup table is not
 * guaranteed to be sparse, the dictionary makes negative indices easier
 * to work with.
 *   TODO: use a threshold to sparsify the cube?
 *
 * The cube is indexed using integer centimeters and degrees since
 * floating points may have rounding errors.
 *
 * Uses a bin resolution of 0.04m x 0.04m x 1°.
 * Uses a fixed domain of [-1m, 1m] x [-1m, 1m] x [-45°, 45°]
 */
class BeliefCube {
  using Point = Eigen::Vector3i;  // [x=dx; y=dy; z=dtheta]

 public:
  BeliefCube() = default;

  /**
   * Generate the belief cube with:
   *  - ref_map: A reference map from a previous time step for
   *      correlative scan matching.
   *  - odom_disp: The odometry displacement data for the motion model.
   *  - odom_angle_disp: The odometry rotation data for the motion model.
   *  - new_obs: New sensor data for correlative scan matching.
   */
  void eval(const RasterMap& ref_map,
            const Eigen::Vector2f& odom_disp,
            const double odom_angle_disp,
            const sensor_msgs::LaserScan& new_obs);

  void eval_range(const RasterMap& ref_map,
                  const Eigen::Vector2f& odom_disp,
                  const double odom_angle_disp,
                  const sensor_msgs::LaserScan& new_obs,
                  const int dtheta_start,
                  const int dtheta_end,
                  const int dx_start,
                  const int dx_end,
                  const int dy_start,
                  const int dy_end);

  /**
   * Return the translational and rotational displacements with the
   * highest probability according to the belief.
   *
   * Return units: [meters, meters, radians].
   */
  std::pair<Eigen::Vector2f, double> max_belief() const;

 private:
  static constexpr int tx_resolution_ = 4;    // centimeters
  static constexpr int rot_resolution_ = 1;   // degrees
  static constexpr int tx_windowsize_ = 100;  // centimeters, inclusive
  static constexpr int rot_windowsize_ = 45;  // degrees, inclusive

  static_assert(tx_windowsize_ % tx_resolution_ == 0);
  static_assert(rot_windowsize_ % rot_resolution_ == 0);

  /**
   * Convert a coordinate space value in meters and radians to a
   * value in the index space.
   */
  int meters_to_tx_index(const double meters) const;
  Point binify(const double x, const double y, const double rad) const;
  Point binify(const Eigen::Vector2f& coord, const double rad) const;

  /**
   * Convert an index value to a coordinate space value in meters
   * and radians.
   */
  double tx_index_to_meters(const int tx_index) const;
  std::pair<Eigen::Vector2f, double> unbinify(const Point& index) const;

 private:
  std::unordered_map<Point, double, util::EigenMatrixHash<Point>> cube_;
  mutable std::optional<std::pair<Eigen::Vector2f, double>> max_belief_;
};

}  // namespace slam

#endif  // SRC_SLAM_BELIEF_CUBE_HH_
