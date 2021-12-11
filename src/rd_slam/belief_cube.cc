#include "rd_slam/belief_cube.hh"
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include "eigen3/Eigen/Dense"
#include "rd_slam/lss.hh"

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

pose_2d::Pose2Df BeliefCube::eval(const SLAMBelief& prev_bel,
                                  const SLAMBelief& cur_bel,
                                  const pose_2d::Pose2Df& odom_disp) {
  const double odom_x = odom_disp.translation.x();
  const double odom_y = odom_disp.translation.y();
  const double odom_th = math_util::ReflexToConvexAngle(odom_disp.angle);

  // Eval halfway in each direction of each delta space
  const double x_delta = std::min(0.01, std::fabs(odom_x) / 10);
  const double y_delta = std::max(0.001, std::fabs(odom_y) / 10);
  const double th_delta = 0.00436332313 / 3;  // 0.25 degree

  double min_error = std::numeric_limits<double>::infinity();
  pose_2d::Pose2Df best_disp;

  for (int x_i = -10; x_i <= 10; x_i++) {
    const double x_disp = odom_x + x_delta * x_i;

    for (int y_i = -10; y_i <= 10; y_i++) {
      const double y_disp = odom_y + y_delta * y_i;

      for (int th_i = -50; th_i <= 50; th_i++) {
        const double th_disp = odom_th + th_delta * th_i;

        const Eigen::Affine2f xform =
            Eigen::Translation2f(x_disp, y_disp) * Eigen::Rotation2Df(th_disp);

        double error_sum = 0;
        for (const auto& corr : cur_bel.corrs_) {
          const auto& ref_line = prev_bel.segments_[corr.prev_i];
          auto hypothesis_line = cur_bel.segments_[corr.cur_i];

          hypothesis_line.p0 = xform * hypothesis_line.p0;
          hypothesis_line.p1 = xform * hypothesis_line.p1;

          error_sum += LSS(ref_line, hypothesis_line, false);
        }

        if (error_sum < min_error) {
          best_disp.Set(th_disp, Eigen::Vector2f(x_disp, y_disp));
          min_error = error_sum;
        }
      }
    }
  }

  return best_disp;
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
