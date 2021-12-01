#include "sensor.hh"

#include <limits>
#include <stdexcept>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "models/constraints.hh"
#include "models/normdist.hh"
#include "sensor_msgs/LaserScan.h"
#include "shared/util/random.h"

namespace {
constexpr double kLidarStddev = 0.08;
constexpr double kGaussianLowerBound = -2.5 * kLidarStddev;
constexpr double kGaussianUpperBound = 2.5 * kLidarStddev;
constexpr double kGamma = 0.12;

const double kLogObsNormalization = models::LnOfNormalPdf(0, 0, kLidarStddev);
util_random::Random rng;
}  // namespace

namespace models {

Observations::Observations(const sensor_msgs::LaserScan& scan,
                           const Eigen::Vector2f& robot_loc,
                           const float robot_angle)
    : min_range_dist_(scan.range_min),
      max_range_dist_(scan.range_max),
      ranges_(),
      robot_loc_(robot_loc),
      robot_angle_(robot_angle) {
  ranges_.reserve(scan.ranges.size());

  // Current behavior: skip invalid ranges
  for (size_t i = 0; i < scan.ranges.size(); i++) {
    const float range_dist = scan.ranges[i];
    if (range_dist <= min_range_dist_ || range_dist >= max_range_dist_) {
      continue;
    }

    const float scan_angle = scan.angle_min + i * scan.angle_increment;
    ranges_.push_back({i, range_dist, scan_angle, true});
  }

  // TODO: How should we initialize this if we keep invalid ranges in the vector?
  point_cloud_ = Eigen::Matrix<float, 2, Eigen::Dynamic>(2, ranges_.size());
  for (size_t i = 0; i < ranges_.size(); i++) {
    const LaserRange& range = ranges_[i];
    point_cloud_.col(i) = Eigen::Rotation2Df(range.angle) * Eigen::Vector2f(range.dist, 0);
  }

  // Right to Left
  const Eigen::Affine2f A_laser_to_map = Eigen::Translation2f(robot_loc_) *
                                         Eigen::Rotation2Df(robot_angle_) *
                                         Eigen::Translation2f(kLaserOffset);
  point_cloud_ = A_laser_to_map * point_cloud_;
}

Observations::Observations(const float min_range_dist,
                           const float max_range_dist,
                           std::vector<LaserRange>&& ranges,
                           const Eigen::Vector2f& robot_loc,
                           const float robot_angle,
                           Eigen::Matrix<float, 2, Eigen::Dynamic>&& point_cloud)
    : min_range_dist_(min_range_dist),
      max_range_dist_(max_range_dist),
      ranges_(ranges),
      robot_loc_(robot_loc),
      robot_angle_(robot_angle),
      point_cloud_(point_cloud) {}

Observations::Observations(const float min_range_dist, const float max_range_dist)
    : min_range_dist_(min_range_dist), max_range_dist_(max_range_dist), ranges_() {}

Observations Observations::density_aware_sample(const double sampling_fraction) const {
  if (sampling_fraction < 0.0 || sampling_fraction > 1.0) {
    throw std::invalid_argument("[Observations::density_aware_sample] invalid sampling fraction");
  }

  Observations sample(min_range_dist_, max_range_dist_);
  sample.robot_loc_ = robot_loc_;
  sample.robot_angle_ = robot_angle_;

  size_t valid_ranges = 0;
  float range_sum = 0;
  float max_valid_range = 0;
  for (const auto& range : ranges_) {
    if (range.valid) {
      valid_ranges++;
      range_sum += range.dist;
      max_valid_range = std::max(max_valid_range, range.dist);
    }
  }

  const size_t target_sample_size = valid_ranges * sampling_fraction;
  const double p_take_max = target_sample_size * max_valid_range / range_sum;
  for (size_t i = 0; i < ranges_.size(); i++) {
    const auto& range = ranges_[i];
    if (!range.valid) {
      continue;
    }

    const float p_take = range.dist / max_valid_range * p_take_max;
    if (rng.UniformRandom() <= p_take) {
      sample.ranges_.push_back(range);
      sample.ranges_.back().msg_idx = i;  // reassign msg_idx for point cloud sampling
    }
  }

  sample.point_cloud_ = Eigen::Matrix<float, 2, Eigen::Dynamic>(2, sample.ranges_.size());
  for (size_t i = 0; i < sample.ranges_.size(); i++) {
    sample.point_cloud_.col(i) = point_cloud_.col(sample.ranges_[i].msg_idx);
  }

  return sample;
}

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
    points.push_back(kLaserOffset + scan_rot * Eigen::Vector2f(scan_range, 0));
  }

  return points;
}

double EvalLogSensorModel(const sensor_msgs::LaserScan& scan_info,
                          const Eigen::Vector2f& expected_obs,
                          const Eigen::Vector2f& sample_obs) {
  const float sample_range = (sample_obs - kLaserOffset).norm();
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
