#ifndef SRC_SLAM_BELIEF_CUBE_HH_
#define SRC_SLAM_BELIEF_CUBE_HH_

#include <optional>
#include <unordered_map>
#include <utility>
#include "eigen3/Eigen/Dense"
#include "models/sensor.hh"
#include "raster_map.hh"
#include "util/matrix_hash.hh"

namespace slam {

/**
 * A discretized representation of the belief (in delta space) at an
 * arbitrary time step.
 *
 * Implemented as a dictionary of keys. Although the lookup table is not
 * guaranteed to be sparse, the dictionary makes negative indices easier
 * to work with.
 *
 * The cube is indexed using integer centimeters and degrees since
 * floating point rounding errors may cause inconsistent insertion
 * and retrieval.
 */
class BeliefCube {
  using Point = Eigen::Vector3i;  // [x=dx; y=dy; z=dtheta]

 public:
  BeliefCube(const int tx_windowsize,
             const int tx_resolution,
             const int rot_windowsize,
             const int rot_resolution);

  /**
   * Generate the belief cube with:
   *  - ref_map: A reference map from a previous time step for
   *      correlative scan matching.
   *  - odom_disp: The odometry displacement data for the motion model.
   *  - odom_angle_disp: The odometry rotation data for the motion model.
   *  - new_obs: New observations for correlative scan matching.
   *  - ignore_motion_model: optimimzation flag to discard motion model
   *      probabilities
   *  - enable_online_obs_pruning: optimization flag to enable
   *      observation likelihood online pruning
   */
  void eval(const RasterMap& ref_map,
            const Eigen::Vector2f& odom_disp,
            const double odom_angle_disp,
            const models::Observations& new_obs,
            const bool ignore_motion_model = false,
            const bool enable_obs_pruning = true);

  void eval_with_coarse(const RasterMap& ref_map,
                        const Eigen::Vector2f& odom_disp,
                        const double odom_angle_disp,
                        const models::Observations& new_obs,
                        const BeliefCube& coarse_cube);

  double eval_range(const RasterMap& ref_map,
                    const Eigen::Vector2f& odom_disp,
                    const double odom_angle_disp,
                    const models::Observations& new_obs,
                    const int dtheta_start,
                    const int dtheta_end,
                    const int dx_start,
                    const int dx_end,
                    const int dy_start,
                    const int dy_end,
                    const bool ignore_motion_model,
                    const bool enable_obs_pruning);

  /**
   * Return the translational and rotational displacements with the
   * highest probability according to the belief.
   *
   * Return units: [meters, meters, radians].
   */
  std::pair<Eigen::Vector2f, double> max_belief() const;

 private:
  decltype(auto) max_index_iterator() const;

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
  const int tx_windowsize_;   // centimeters, inclusive, symmetric
  const int tx_resolution_;   // centimeters
  const int rot_windowsize_;  // degrees, inclusive, symmetric
  const int rot_resolution_;  // degrees
  std::unordered_map<Point, double, util::EigenMatrixHash<Point>> cube_;
  mutable std::optional<std::pair<Eigen::Vector2f, double>> max_belief_;
};

}  // namespace slam

#endif  // SRC_SLAM_BELIEF_CUBE_HH_
