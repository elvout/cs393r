#include "sensor.hh"

#include <limits>
#include "config_reader/config_reader.h"
#include "eigen3/Eigen/Dense"
#include "models/constraints.hh"
#include "models/normdist.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
CONFIG_DOUBLE(LidarStddev, "lidar_stddev");
CONFIG_DOUBLE(GaussianLowerBound, "sensor_model_d_short");
CONFIG_DOUBLE(GaussianUpperBound, "sensor_model_d_long");
CONFIG_DOUBLE(Gamma, "sensor_model_gamma");

const double kLogObsNormalization = models::LnOfNormalPdf(0, 0, CONFIG_LidarStddev);
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
  return LnOfNormalPdf(range_diff, 0, CONFIG_LidarStddev) - kLogObsNormalization;
}

double RobustLogSensorModelThreshold(const double stddevs) {
  return LnOfNormalPdf(stddevs * CONFIG_LidarStddev, 0, CONFIG_LidarStddev) - kLogObsNormalization;
}

}  // namespace models
