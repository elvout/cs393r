#ifndef SRC_RD_SLAM_BELIEF_CUBE_HH_
#define SRC_RD_SLAM_BELIEF_CUBE_HH_

#include <optional>
#include <unordered_map>
#include <utility>
#include "eigen3/Eigen/Dense"
#include "rd_slam/rd_slam.hh"
#include "util/matrix_hash.hh"

namespace rd_slam {

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
   * Generate the belief cube with the segment correspondences of:
   *  - prev_bel
   *  - cur_bel
   *  - odom_disp
   * in the delta-space around odom_disp
   * by minimizing the total error for all line segments from a perturbation.
   */
  pose_2d::Pose2Df eval(const SLAMBelief& prev_bel,
                        const SLAMBelief& cur_bel,
                        const pose_2d::Pose2Df& odom_disp);

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

}  // namespace rd_slam

#endif  // SRC_RD_SLAM_BELIEF_CUBE_HH_
