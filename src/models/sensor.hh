#ifndef SRC_MODELS_SENSOR_HH_
#define SRC_MODELS_SENSOR_HH_

#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"

namespace models {

/**
 * Evaluate the log-likelihood of the observation likelihood model.
 */
double EvalLogSensorModel(const sensor_msgs::LaserScan& scan_info,
                          const Eigen::Vector2f& expected_obs,
                          const Eigen::Vector2f& sample_obs);

/**
 * Log-likelihood threshold for the simple robust sensor model.
 */
double RobustLogSensorModelThreshold(const double stddevs);

}  // namespace models

#endif  // SRC_MODELS_SENSOR_HH_
