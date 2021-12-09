#include "rd_slam/belief_cube.hh"
#include <stdexcept>
#include <unordered_set>
#include "eigen3/Eigen/Dense"
#include "rd_slam/rd_slam.hh"

namespace rd_slam {
BeliefCube::BeliefCube(const int tx_windowsize,
                       const int tx_resolution,
                       const int rot_windowsize,
                       const int rot_resolution)
    : tx_windowsize_(tx_windowsize),
      tx_resolution_(tx_resolution),
      rot_windowsize_(rot_windowsize),
      rot_resolution_(rot_resolution),
      cube_(),
      max_belief_() {
  if (tx_windowsize_ % tx_resolution_ != 0) {
    throw std::invalid_argument("[BeliefCube] tx windowsize, resolution mismatch");
  }

  if (rot_windowsize_ % rot_resolution_ != 0) {
    throw std::invalid_argument("[BeliefCube] rot windowsize, resolution mismatch");
  }
}

decltype(auto) BeliefCube::max_index_iterator() const {
  if (cube_.empty()) {
    throw std::runtime_error("[BeliefCube::max_index_iterator() FATAL]: empty cube");
  }

  auto it = cube_.begin();
  auto max_it = it;
  while (++it != cube_.cend()) {
    if (it->second > max_it->second) {
      max_it = it;
    }
  }
  return max_it;
}

void BeliefCube::eval(const SLAMBelief& prev_bel,
                      const SLAMBelief& cur_bel,
                      const pose_2d::Pose2Df odom_disp) {
  for (int dtheta = -rot_windowsize_; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
    const int dtheta_index = dtheta / rot_resolution_;

    for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
      const int dx_index = dx / tx_resolution_;

      // for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
      int dy = 0;
      const int dy_index = dy / tx_resolution_;

      const Point index(dx_index, dy_index, dtheta_index);
      const auto [hypothesis_disp, hypothesis_rot] = unbinify(index);

      const Eigen::Affine2f xform =
          Eigen::Translation2f(hypothesis_disp) * Eigen::Rotation2Df(hypothesis_rot);

      // double error_sum = (hypothesis_disp - odom_disp.translation).norm();
      double error_sum = 0;

      for (const auto& corr : cur_bel.corrs_) {
        const auto& ref_line = prev_bel.segments_[corr.prev_i];
        auto hypothesis_line = cur_bel.segments_[corr.cur_i];

        hypothesis_line.p0 = xform * hypothesis_line.p0;
        hypothesis_line.p1 = xform * hypothesis_line.p1;

        error_sum += LSS(ref_line, hypothesis_line, false);
      }

      // we aim to minimize the error sum
      cube_.emplace(index, -error_sum);
      // }
    }
  }
}

std::pair<Eigen::Vector2f, double> BeliefCube::max_belief() const {
  if (max_belief_.has_value()) {
    return max_belief_.value();
  }

  max_belief_ = unbinify(max_index_iterator()->first);
  printf("%.4f\n", -max_index_iterator()->second);
  return max_belief_.value();
}

int BeliefCube::meters_to_tx_index(const double meters) const {
  // TODO: breaks ties towards positive inf
  // It should probably be either towards or away from 0 instead
  // for positive/negative symmetry.

  const double cm = meters * 100.0;
  const double bin = (cm + tx_resolution_ * 0.5) / tx_resolution_;
  return static_cast<int>(bin);
}

BeliefCube::Point BeliefCube::binify(const double x, const double y, const double rad) const {
  using math_util::RadToDeg;
  using math_util::ReflexToConvexAngle;

  const int x_bin = meters_to_tx_index(x);
  const int y_bin = meters_to_tx_index(y);

  // TOOD: round instead of floor?
  const int theta_bin = static_cast<int>(RadToDeg(ReflexToConvexAngle(rad)));
  return Point(x_bin, y_bin, theta_bin);
}

BeliefCube::Point BeliefCube::binify(const Eigen::Vector2f& coord, const double rad) const {
  return binify(coord.x(), coord.y(), rad);
}

double BeliefCube::tx_index_to_meters(const int tx_index) const {
  const double cm = tx_index * tx_resolution_;
  return cm / 100.0;
}

std::pair<Eigen::Vector2f, double> BeliefCube::unbinify(const Point& index) const {
  const double x_coord = tx_index_to_meters(index.x());
  const double y_coord = tx_index_to_meters(index.y());

  // potential for a bug:
  // we don't account for rot_resolution in binify nor here
  const double rad = math_util::DegToRad(static_cast<double>(index.z()));

  return std::make_pair(Eigen::Vector2f(x_coord, y_coord), rad);
}
}  // namespace rd_slam
