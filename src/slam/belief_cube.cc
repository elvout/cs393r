#include "belief_cube.hh"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math/math_util.h"
#include "models.hh"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);
}  // namespace

std::vector<Eigen::Vector2f> correlations(const RasterMap& ref_map,
                                          const sensor_msgs::LaserScan& obs,
                                          const Eigen::Vector2f& bel_disp,
                                          const double bel_rot) {
  std::vector<Eigen::Vector2f> obs_points = PointsFromScan(obs);
  std::vector<Eigen::Vector2f> correlations;

  const Eigen::Rotation2Df dtheta_rot(bel_rot);
  for (size_t scan_i = 0; scan_i < obs_points.size(); scan_i++) {
    const Eigen::Vector2f& point = obs_points[scan_i];
    const Eigen::Vector2f query_point = dtheta_rot * point + bel_disp;

    const float query_dist = query_point.norm();
    if (query_dist <= obs.range_min || query_dist >= obs.range_max) {
      continue;
    }

    double log_obs_prob = ref_map.query(query_point.x(), query_point.y());
    if (log_obs_prob != -std::numeric_limits<double>::infinity()) {
      correlations.push_back(point);
    }
  }

  return correlations;
}

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const sensor_msgs::LaserScan& new_obs) {
  cube_.clear();
  max_belief_.reset();

  // Evaluate the motion model first to prune the index space a bit.
  size_t evals = 0;
  // not a great threshold, as the standard devations are dependent on the magnitude
  // of displacement
  const double log_prob_threshold = LogNormalPdf(3, 0, 1);
  for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
    const int dtheta_index = dtheta / rot_resolution_;

    for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
      const int dx_index = dx / tx_resolution_;

      for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
        const int dy_index = dy / tx_resolution_;
        evals++;

        const Point index(dx_index, dy_index, dtheta_index);
        const auto [hypothesis_disp, hypothesis_rot] = unbinify(index);

        const double log_motion_prob =
            LogMotionModel(odom_disp, odom_angle_disp, hypothesis_disp, hypothesis_rot);

        if (log_motion_prob > log_prob_threshold) {
          cube_[index] = log_motion_prob;
        }
      }
    }
  }

  printf("[BeliefCube::eval INFO] cube size: %lu / %lu\n", cube_.size(), evals);

  // Evaluate the observation likelihood model on plausible indices
  // according to the motion model.
  // 2D slicing was too slow; maybe I was just doing it wrong
  const std::vector<Eigen::Vector2f> obs_points = PointsFromScan(new_obs);
  for (auto it = cube_.begin(); it != cube_.cend(); it++) {
    const Point& index = it->first;

    const auto [d_loc, d_theta] = unbinify(index);
    const Eigen::Rotation2Df dtheta_rot(d_theta);
    double log_sum = 0;

    for (size_t scan_i = 0; scan_i < obs_points.size(); scan_i += 10) {
      const Eigen::Vector2f& point = obs_points[scan_i];

      // Translate the observation into the previous reference frame.
      const Eigen::Vector2f query_point = dtheta_rot * point + d_loc;
      const float query_dist = query_point.norm();
      if (query_dist <= new_obs.range_min || query_dist >= new_obs.range_max) {
        continue;
      }

      double log_obs_prob = ref_map.query(query_point.x(), query_point.y());
      // approximate the symmetric robust observation likelihood model
      // TODO: un-hardcode, 2.5 stddev was used in particle filter
      log_obs_prob = std::max(log_obs_prob, -2.0);
      log_sum += log_obs_prob;
    }

    // TODO: prune out impossible values to sparsify matrix?

    // TODO: parameterize gamma
    it->second += log_sum * 0.12;
  }
}

std::pair<Eigen::Vector2f, double> BeliefCube::max_belief() const {
  if (max_belief_.has_value()) {
    return max_belief_.value();
  }

  if (cube_.empty()) {
    throw std::runtime_error("[BeliefCube::max_belief() FATAL]: empty cube");
  }

  auto it = cube_.begin();
  auto max_index = it;

  while (++it != cube_.cend()) {
    if (it->second > max_index->second) {
      max_index = it;
    }
  }

  max_belief_ = unbinify(max_index->first);
  return max_belief_.value();
}

int BeliefCube::meters_to_tx_index(const double meters) const {
  // TODO: breaks ties towards positive inf
  // It should probably be either towards or away from 0 instead
  // for positive/negative symmetry.

  const double cm = meters * 100.0;
  const double bin = (cm + tx_resolution_ * 0.5) / tx_resolution_;
  return static_cast<int>(bin);
}

BeliefCube::Point BeliefCube::binify(const double x, const double y, const double rad) const {
  using math_util::ConstrainAngle;
  using math_util::RadToDeg;

  const int x_bin = meters_to_tx_index(x);
  const int y_bin = meters_to_tx_index(y);

  // TOOD: round instead of floor?
  const int theta_bin = static_cast<int>(RadToDeg(ConstrainAngle(rad)));
  return Point(x_bin, y_bin, theta_bin);
}

BeliefCube::Point BeliefCube::binify(const Eigen::Vector2f& coord, const double rad) const {
  return binify(coord.x(), coord.y(), rad);
}

double BeliefCube::tx_index_to_meters(const int tx_index) const {
  const double cm = tx_index * tx_resolution_;
  return cm / 100.0;
}

std::pair<Eigen::Vector2f, double> BeliefCube::unbinify(const Point& index) const {
  const double x_coord = tx_index_to_meters(index.x());
  const double y_coord = tx_index_to_meters(index.y());

  // potential for a bug:
  // we don't account for rot_resolution in binify nor here
  const double rad = math_util::DegToRad(static_cast<double>(index.z()));

  return std::make_pair(Eigen::Vector2f(x_coord, y_coord), rad);
}
