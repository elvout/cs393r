#ifndef SRC_SLAM_MODELS_HH_
#define SRC_SLAM_MODELS_HH_

#include <vector>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"

std::vector<Eigen::Vector2f> PointsFromScan(const sensor_msgs::LaserScan& scan);

/**
 * Return the log-likelihood of the Normal PDF.
 */
double LogNormalPdf(const double val, const double mean, const double stddev);

/**
 * Return the log-likelihood of the motion model.
 */
double LogMotionModel(const Eigen::Vector2f& expected_disp,
                      const double expected_rot,
                      const Eigen::Vector2f& hypothesis_disp,
                      const double hypothesis_rot);

/**
 * Return the log-likelihood of the observation likelihood model.
 */
double LogObsModel(const sensor_msgs::LaserScan& obs,
                   const Eigen::Vector2f& expected,
                   const Eigen::Vector2f& hypothesis);

#endif  // SRC_SLAM_MODELS_HH_
