#ifndef SRC_MODELS_SENSOR_HH_
#define SRC_MODELS_SENSOR_HH_

#include <vector>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"

namespace models {

/**
 * Return a point cloud of observations in the base link reference
 * frame of the car from the valid sensor observations in the scan
 * message.
 */
std::vector<Eigen::Vector2f> PointsFromScan(const sensor_msgs::LaserScan& scan);

/**
 * Evaluate the log-likelihood of the observation likelihood model.
 * The observation points should be in the base link reference frame of
 * the car.
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
