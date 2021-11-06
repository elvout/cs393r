#include "belief_cube.hh"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include "common.hh"
#include "eigen3/Eigen/Dense"
#include "math/math_util.h"
#include "models.hh"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

namespace slam {

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const sensor_msgs::LaserScan& new_obs) {
  auto __delayedfn = common::runtime_dist().auto_lap("BeliefCube::eval");
  cube_.clear();
  max_belief_.reset();

  common::runtime_dist().start_lap("BeliefCube::eval (Motion Model)");
  // Evaluate the motion model first to prune the index space a bit.
  size_t evals = 0;
  // not a great threshold, as the standard devations are dependent on the magnitude
  // of displacement
  const double log_prob_threshold = LogNormalPdf(3, 0, 1);
  for (int dtheta = -rot_windowsize_; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
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
  common::runtime_dist().end_lap("BeliefCube::eval (Motion Model)");

  printf("[BeliefCube::eval INFO] cube size: %lu / %lu\n", cube_.size(), evals);

  common::runtime_dist().start_lap("BeliefCube::eval (Observation Model)");

  // Evaluate the observation likelihood model on plausible indices
  // according to the motion model.
  // Since the only care about the index with the maximum likelihood,
  // we can keep track of the running maximium likelihood and prune
  // while evaluating the model if we know that the current likelihood
  // cannot be greater than the maximum likelihood.
  const std::vector<Eigen::Vector2f> obs_points = PointsFromScan(new_obs);
  size_t prune_count = 0;
  double max_prob = -std::numeric_limits<double>::infinity();

  for (auto it = cube_.begin(); it != cube_.cend();) {
    const Point& index = it->first;

    const auto [d_loc, d_theta] = unbinify(index);
    const Eigen::Rotation2Df dtheta_rot(d_theta);
    double log_sum = 0;

    // CRITICAL PRUNING ASSUMPTION: ref_map.query (LogObsModel) is never positive
    const double prune_threshold = max_prob - it->second;
    bool prune = false;

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

      if (log_sum < prune_threshold) {
        prune = true;
        break;
      }
    }

    if (prune) {
      it = cube_.erase(it);
      prune_count++;
    } else {
      it->second += log_sum;
      max_prob = std::max(max_prob, it->second);
      it++;
    }
  }
  common::runtime_dist().end_lap("BeliefCube::eval (Observation Model)");

  printf("[BeliefCube::eval]: pruned %lu entries\n", prune_count);
}

std::pair<Eigen::Vector2f, double> BeliefCube::max_belief() const {
  auto __delayedfn = common::runtime_dist().auto_lap("BeliefCube::max_belief");
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
  using math_util::RadToDeg;
  using math_util::ReflexToConvexAngle;

  const int x_bin = meters_to_tx_index(x);
  const int y_bin = meters_to_tx_index(y);

  // TOOD: round instead of floor?
  const int theta_bin = static_cast<int>(RadToDeg(ReflexToConvexAngle(rad)));
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

}  // namespace slam
