#include "sensor.hh"

#include <limits>
#include "eigen3/Eigen/Dense"
#include "models/constraints.hh"
#include "models/normdist.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
constexpr double kLidarStddev = 0.1;
constexpr double kGaussianLowerBound = -2.5 * kLidarStddev;
constexpr double kGaussianUpperBound = 2.5 * kLidarStddev;
constexpr double kGamma = 0.12;

const double kLogObsNormalization = models::LnOfNormalPdf(0, 0, kLidarStddev);
}  // namespace

namespace models {
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

double RobustLogSensorModelThreshold(const double stddevs) {
  return LnOfNormalPdf(stddevs * kLidarStddev, 0, kLidarStddev) - kLogObsNormalization;
}

}  // namespace models
