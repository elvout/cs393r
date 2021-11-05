#include "belief_cube.hh"
#include <iostream>
#include <limits>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math/math_util.h"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);

std::vector<Eigen::Vector2f> PointsFromScan(const sensor_msgs::LaserScan& scan) {
  const std::vector<float>& ranges = scan.ranges;
  const size_t n_ranges = ranges.size();

  std::vector<Eigen::Vector2f> points;
  points.reserve(n_ranges);

  for (size_t i = 0; i < n_ranges; i++) {
    const float scan_range = ranges[i];
    if (scan_range <= scan.range_min || scan_range >= scan.range_max) {
      continue;
    }

    const float scan_angle = scan.angle_min + i * scan.angle_increment;
    const Eigen::Rotation2Df scan_rot(scan_angle);
    points.push_back(laser_loc + scan_rot * Eigen::Vector2f(scan_range, 0));
  }

  return points;
}

// TODO: refactor, move to shared or util library
double NormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2pi))
  constexpr double inv_sqrt2pi = 0.39894228040143267794;

  double z = (val - mean) / stddev;
  return inv_sqrt2pi / stddev * std::exp(-0.5 * z * z);
}

double MotionModel(const Eigen::Vector2f& reference_point,
                   const double reference_angle,
                   const Eigen::Vector2f& expected_tx,
                   const double expected_rot,
                   const Eigen::Vector2f& hypothesis_tx,
                   const double hypothesis_rot) {
  // calculate displacement in the reference frame
  const Eigen::Rotation2Df ref_rot(-reference_angle);
  const Eigen::Vector2f expected_disp = ref_rot * (expected_tx - reference_point);
  const Eigen::Vector2f hypothesis_disp = ref_rot * (hypothesis_tx - reference_point);

  const double expected_angular_disp = math_util::ConstrainAngle(expected_rot - reference_angle);
  const double hypothesis_angular_disp =
      math_util::ConstrainAngle(hypothesis_rot - reference_angle);

  {
    // sanity check
    const double exp_ang_disp_check =
        math_util::ConstrainAngle(std::atan2(expected_disp.y(), expected_disp.x()));
    const double hyp_ang_disp_check =
        math_util::ConstrainAngle(std::atan2(hypothesis_disp.y(), hypothesis_disp.x()));

    if (std::abs(expected_angular_disp - exp_ang_disp_check) >= 1e-5) {
      std::cerr << "[belief_cube::MotionModel WARNING] expected angular displacement mismatch\n";
    }

    if (std::abs(hypothesis_angular_disp - hyp_ang_disp_check) >= 1e-5) {
      std::cerr << "[belief_cube::MotionModel WARNING] hypothesis angular displacement mismatch\n";
    }
  }

  // TODO: move params to config?
  constexpr double k1 = 0.45;
  constexpr double k2 = 1.6;
  constexpr double k3 = 0.65;
  constexpr double k4 = 2.3;

  const double x_noise = hypothesis_disp.x() - expected_disp.x();
  const double y_noise = hypothesis_disp.y() - expected_disp.y();
  const double theta_noise =
      math_util::ReflexToConvexAngle(hypothesis_angular_disp - expected_angular_disp);

  const double expected_disp_n = expected_disp.norm();
  const double expected_rot_n = std::abs(math_util::ReflexToConvexAngle(expected_angular_disp));
  const double tx_std = k1 * expected_disp_n + k2 * expected_rot_n;
  const double rot_std = k3 * expected_disp_n + k4 * expected_rot_n;

  const double px = NormalPdf(x_noise, 0, tx_std);
  const double py = NormalPdf(y_noise, 0, tx_std);
  const double ptheta = NormalPdf(theta_noise, 0, rot_std);

  // todo: log-likelihoods?
  return px * py * ptheta;
}

}  // namespace

// in raster_map we compute the map in the constructor
// should we make the APIs consistent?
BeliefCube::BeliefCube() : cube_() {}

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& ref_loc,
                      const double ref_angle,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const sensor_msgs::LaserScan& new_obs) {
  const std::vector<Eigen::Vector2f> obs_points = PointsFromScan(new_obs);

  // observation likelihood model
  for (const Eigen::Vector2f& point : obs_points) {
    for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
      // negative?
      const Eigen::Rotation2Df dtheta_rot(math_util::DegToRad(static_cast<double>(dtheta)));
      Eigen::Vector2f proj_point = dtheta_rot * point;

      for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
        const double query_x = proj_point.x() + (dx / 100.0);  // meters
        for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
          const double query_y = proj_point.y() + (dy / 100.0);  // meters

          double obs_prob = ref_map.query(query_x, query_y);
          Point key = binify(dx, dy, dtheta);
          cube_[key] += std::log(obs_prob);
        }
      }
    }
  }

  // motion model
  // iterate over non-zero points
  for (auto it = cube_.begin(); it != cube_.cend();) {
    if (it->second == -std::numeric_limits<double>::infinity()) {
      it = cube_.erase(it);
    } else {
      it++;
    }
  }

  const Eigen::Vector2f expected_loc = ref_loc + odom_disp;
  const double expected_angle = ref_angle + odom_angle_disp;
  for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
    for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
      for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
        Point key(dx, dy, dtheta);
        auto it = cube_.find(key);
        if (it == cube_.cend()) {
          continue;
        }

        Eigen::Vector2f tx_delta(dx / 100.0, dy / 100.0);
        Eigen::Vector2f hypothesis_loc = ref_loc + tx_delta;
        double hypothesis_angle = ref_angle + dtheta;

        double motion_prob = MotionModel(ref_loc, ref_angle, expected_loc, expected_angle,
                                         hypothesis_loc, hypothesis_angle);
        double log_prob = std::log(motion_prob);

        if (log_prob == -std::numeric_limits<double>::infinity()) {
          cube_.erase(it);
        } else {
          it->second += log_prob;
        }
      }
    }
  }
}

BeliefCube::Point BeliefCube::binify(const double x, const double y, const double rad) const {
  using math_util::ConstrainAngle;
  using math_util::RadToDeg;

  double x_bin = (x * 100.0 + tx_resolution_ * 0.5) / tx_resolution_;
  double y_bin = (y * 100.0 + tx_resolution_ * 0.5) / tx_resolution_;
  double theta_bin = RadToDeg(ConstrainAngle(rad));

  return Point(static_cast<int>(x_bin), static_cast<int>(y_bin), static_cast<int>(theta_bin));
}

BeliefCube::Point BeliefCube::binify(const Eigen::Vector2f& coord, const double rad) const {
  return binify(coord.x(), coord.y(), rad);
}
