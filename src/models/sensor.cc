#include "sensor.hh"

#include <limits>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "models/constraints.hh"
#include "models/normdist.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
constexpr double kLidarStddev = 0.08;
constexpr double kGaussianLowerBound = -2.5 * kLidarStddev;
constexpr double kGaussianUpperBound = 2.5 * kLidarStddev;
constexpr double kGamma = 0.12;

const double kLogObsNormalization = models::LnOfNormalPdf(0, 0, kLidarStddev);
}  // namespace

namespace models {

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
    points.push_back(kLaserLoc + scan_rot * Eigen::Vector2f(scan_range, 0));
  }

  return points;
}

double EvalLogSensorModel(const sensor_msgs::LaserScan& scan_info,
                          const Eigen::Vector2f& expected_obs,
                          const Eigen::Vector2f& sample_obs) {
  const float sample_range = (sample_obs - kLaserLoc).norm();
  if (sample_range <= scan_info.range_min || sample_range >= scan_info.range_max) {
    return -std::numeric_limits<double>::infinity();
  }

  const float range_diff = (expected_obs - sample_obs).norm();
  return LnOfNormalPdf(range_diff, 0, kLidarStddev) - kLogObsNormalization;
}

double EvalLogSensorModel(const float expected_range, const float sample_range) {
  const float range_diff = expected_range - sample_range;
  return LnOfNormalPdf(range_diff, 0, kLidarStddev) - kLogObsNormalization;
}

double RobustLogSensorModelThreshold(const double stddevs) {
  return LnOfNormalPdf(stddevs * kLidarStddev, 0, kLidarStddev) - kLogObsNormalization;
}

}  // namespace models
