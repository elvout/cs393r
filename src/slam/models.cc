#include "models.hh"
#include <cmath>
#include <limits>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "shared/math/math_util.h"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);

// TODO: move params to config?
constexpr double k1 = 0.45;
constexpr double k2 = 1.6;
constexpr double k3 = 0.65;
constexpr double k4 = 2.3;

constexpr double kLidarStddev = 0.08;
}  // namespace

namespace slam {

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

double LogNormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for log(1 / sqrt(2pi))
  constexpr double log_inv_sqrt2pi = -0.91893853320467274178;

  const double z = (val - mean) / stddev;
  return log_inv_sqrt2pi - std::log(stddev) - (z * z * 0.5);
}

double LogMotionModel(const Eigen::Vector2f& expected_disp,
                      const double expected_rot,
                      const Eigen::Vector2f& hypothesis_disp,
                      const double hypothesis_rot) {
  const double expected_disp_n = expected_disp.norm();
  const double expected_rot_n = std::abs(math_util::ReflexToConvexAngle(expected_rot));
  const double disp_std = k1 * expected_disp_n + k2 * expected_rot_n;
  const double rot_std = k3 * expected_disp_n + k4 * expected_rot_n;

  const double x_noise = hypothesis_disp.x() - expected_disp.x();
  const double y_noise = hypothesis_disp.y() - expected_disp.y();
  const double theta_noise = math_util::ReflexToConvexAngle(hypothesis_rot - expected_rot);

  const double log_px = LogNormalPdf(x_noise, 0, disp_std);
  const double log_py = LogNormalPdf(y_noise, 0, disp_std);
  const double log_ptheta = LogNormalPdf(theta_noise, 0, rot_std);

  return log_px + log_py + log_ptheta;
}

double LogObsModel(const sensor_msgs::LaserScan& obs,
                   const Eigen::Vector2f& expected,
                   const Eigen::Vector2f& hypothesis) {
  float hypothesis_range = (hypothesis - laser_loc).norm();
  if (hypothesis_range <= obs.range_min || hypothesis_range >= obs.range_max) {
    return -std::numeric_limits<double>::infinity();
  }

  float range_diff = (expected - hypothesis).norm();
  return LogNormalPdf(range_diff, 0, kLidarStddev);
}

}  // namespace slam
