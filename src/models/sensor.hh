#ifndef SRC_MODELS_SENSOR_HH_
#define SRC_MODELS_SENSOR_HH_

#include <vector>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"

namespace models {

/**
 * A unified format for sensor Observations.
 *
 * Stores valid ranges in the laser reference frame and their
 * corresponding points in the map frame.
 */
class Observations {
 public:
  struct LaserRange {
    size_t msg_idx;
    float dist;
    float angle;
    bool valid;
  };

 public:
  /**
   * Initialize an Observations instance from a LaserScan message.
   *
   * The default parameters for robot_loc and robot_angle construct the
   * point cloud in the base_link reference frame.
   */
  Observations(const sensor_msgs::LaserScan& scan,
               const Eigen::Vector2f& robot_loc = Eigen::Vector2f(0, 0),
               const float robot_angle = 0);

  /**
   * Fake move-constructor in lieu of friendship.
   */
  Observations(const float min_range_dist,
               const float max_range_dist,
               std::vector<LaserRange>&& ranges,
               const Eigen::Vector2f& robot_loc,
               const float robot_angle,
               Eigen::Matrix<float, 2, Eigen::Dynamic>&& point_cloud);

  const std::vector<LaserRange>& ranges() const { return ranges_; }
  const Eigen::Matrix<float, 2, Eigen::Dynamic>& point_cloud() const { return point_cloud_; }

  /**
   * Return a density-aware sample of this Observations instance with
   * the specified sampling fraction.
   */
  Observations density_aware_sample(const double sampling_fraction = 0.1) const;

 private:
  Observations(const float min_range_dist, const float max_range_dist);

 public:
  const float min_range_dist_;
  const float max_range_dist_;

 private:
  // Sensor readings in the laser reference frame.
  std::vector<LaserRange> ranges_;

  // Pose of the robot in the map frame at observation time.
  Eigen::Vector2f robot_loc_;
  float robot_angle_;

  /**
   * Point cloud representation of valid ranges in the map frame.
   * If robot_loc_ and robot_angle_ are both zero, the point cloud
   *   is in the base_link reference frame.
   */
  Eigen::Matrix<float, 2, Eigen::Dynamic> point_cloud_;
};

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
 * Same as above, but assume the the two ranges are valid and have the
 * same scan angle.
 */
double EvalLogSensorModel(const float expected_range, const float sample_range);

/**
 * Log-likelihood threshold for the simple robust sensor model.
 */
double RobustLogSensorModelThreshold(const double stddevs);

}  // namespace models

#endif  // SRC_MODELS_SENSOR_HH_
