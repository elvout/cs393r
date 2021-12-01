#include "belief_cube.hh"
#include <future>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include "common/common.hh"
#include "eigen3/Eigen/Dense"
#include "math/math_util.h"
#include "models/motion.hh"
#include "models/normdist.hh"
#include "models/sensor.hh"
#include "raster_map.hh"
#include "util/matrix_hash.hh"

namespace slam {

BeliefCube::BeliefCube(const int tx_windowsize,
                       const int tx_resolution,
                       const int rot_windowsize,
                       const int rot_resolution)
    : tx_windowsize_(tx_windowsize),
      tx_resolution_(tx_resolution),
      rot_windowsize_(rot_windowsize),
      rot_resolution_(rot_resolution) {
  if (tx_windowsize_ % tx_resolution_ != 0) {
    throw std::invalid_argument("[BeliefCube] tx windowsize, resolution mismatch");
  }

  if (rot_windowsize_ % rot_resolution_ != 0) {
    throw std::invalid_argument("[BeliefCube] rot windowsize, resolution mismatch");
  }
}

decltype(auto) BeliefCube::max_index_iterator() const {
  if (cube_.empty()) {
    throw std::runtime_error("[BeliefCube::max_index_iterator() FATAL]: empty cube");
  }

  auto it = cube_.begin();
  auto max_it = it;
  while (++it != cube_.cend()) {
    if (it->second > max_it->second) {
      max_it = it;
    }
  }
  return max_it;
}

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const models::Observations& new_obs,
                      const bool ignore_motion_model,
                      const bool enable_obs_pruning) {
  auto __delayedfn = common::runtime_dist.auto_lap("BeliefCube::eval");
  cube_.clear();
  max_belief_.reset();

  eval_range(ref_map, odom_disp, odom_angle_disp, new_obs, -rot_windowsize_, rot_windowsize_ + 1,
             -tx_windowsize_, tx_windowsize_ + 1, -tx_windowsize_, tx_windowsize_ + 1,
             ignore_motion_model, enable_obs_pruning);
}

// TODO: refactor
struct Entry {
  Eigen::Vector3i index;
  double prob;

  Entry(const Eigen::Vector3i& i, double p) : index(i), prob(p) {}

  bool operator<(const Entry& other) const { return prob < other.prob; }
};

void BeliefCube::eval_with_coarse(const RasterMap& ref_map,
                                  const Eigen::Vector2f& odom_disp,
                                  const double odom_angle_disp,
                                  const models::Observations& new_obs,
                                  const BeliefCube& coarse_cube) {
  auto __delayedfn = common::runtime_dist.auto_lap("BeliefCube::eval_with_coarse");

  // these checks probably aren't strictly necessary
  // TODO: should also be even multiples (coarse res should be even)
  if (tx_windowsize_ != coarse_cube.tx_windowsize_ ||
      coarse_cube.tx_resolution_ % tx_resolution_ != 0 ||
      rot_windowsize_ != coarse_cube.rot_windowsize_ ||
      coarse_cube.rot_resolution_ % rot_resolution_ != 0) {
    throw std::invalid_argument("[BeliefCube::eval_with_coarse] windowsize/resolution mismatch");
  }

  std::priority_queue<Entry> coarse_entries;
  for (const auto& [key, val] : coarse_cube.cube_) {
    coarse_entries.emplace(key, val);
  }

  std::unordered_set<Eigen::Vector2i, util::EigenMatrixHash<Eigen::Vector2i>> visited;

  double max_fine_bel_prob = -std::numeric_limits<double>::infinity();
  while (!coarse_entries.empty()) {
    Entry coarse_entry = coarse_entries.top();
    coarse_entries.pop();

    const Eigen::Vector2i coarse_tx_index(coarse_entry.index.x(), coarse_entry.index.y());
    if (visited.count(coarse_tx_index) == 1) {
      continue;
    }
    visited.insert(coarse_tx_index);

    if (max_fine_bel_prob > 0) {
      throw std::runtime_error("BeliefCube::eval_with_coarse: log probability positive");
    }

    if (coarse_entry.prob < max_fine_bel_prob) {
      break;
    }

    // convert coarse indices to discrete centimeters
    const int dx_start =
        coarse_entry.index.x() * coarse_cube.tx_resolution_ - coarse_cube.tx_resolution_ / 2;
    const int dx_end =
        coarse_entry.index.x() * coarse_cube.tx_resolution_ + coarse_cube.tx_resolution_ / 2 + 1;
    const int dy_start =
        coarse_entry.index.y() * coarse_cube.tx_resolution_ - coarse_cube.tx_resolution_ / 2;
    const int dy_end =
        coarse_entry.index.y() * coarse_cube.tx_resolution_ + coarse_cube.tx_resolution_ / 2 + 1;

    // const int dtheta_start =
    //     coarse_entry.index.z() * coarse_cube.rot_resolution_ - coarse_cube.rot_resolution_ / 2;
    // const int dtheta_end =
    //     coarse_entry.index.z() * coarse_cube.rot_resolution_ + coarse_cube.rot_resolution_ / 2 +
    //     1;

    // multi-res seems to get the angle wrong pretty often so evaluate all the angles
    // use  the visited set to avoid recomputations
    const int dtheta_start = -rot_windowsize_;
    const int dtheta_end = rot_windowsize_ + 1;

    double fine_bel_prob = eval_range(ref_map, odom_disp, odom_angle_disp, new_obs, dtheta_start,
                                      dtheta_end, dx_start, dx_end, dy_start, dy_end, false, true);
    max_fine_bel_prob = std::max(max_fine_bel_prob, fine_bel_prob);
  }
}

double BeliefCube::eval_range(const RasterMap& ref_map,
                              const Eigen::Vector2f& odom_disp,
                              const double odom_angle_disp,
                              const models::Observations& new_obs,
                              const int dtheta_start,
                              const int dtheta_end,
                              const int dx_start,
                              const int dx_end,
                              const int dy_start,
                              const int dy_end,
                              const bool ignore_motion_model,
                              const bool enable_obs_pruning) {
  // not a great threshold, as the standard deviations are dependent
  // on the magnitudes of displacement
  static const double log_motion_prob_threshold = models::LnOfNormalPdf(3, 0, 1);

  // approximate the symmetric robust observation likelihood model
  static const double log_obs_prob_threshold = models::RobustLogSensorModelThreshold(2.5);

  std::priority_queue<Entry> plausible_entries;

  // Evaluate the motion model first to prune the index space a bit.
  for (int dtheta = dtheta_start; dtheta < dtheta_end; dtheta += rot_resolution_) {
    const int dtheta_index = dtheta / rot_resolution_;

    for (int dx = dx_start; dx < dx_end; dx += tx_resolution_) {
      const int dx_index = dx / tx_resolution_;

      for (int dy = dy_start; dy < dy_end; dy += tx_resolution_) {
        const int dy_index = dy / tx_resolution_;

        const Point index(dx_index, dy_index, dtheta_index);
        const auto [hypothesis_disp, hypothesis_rot] = unbinify(index);

        const double log_motion_prob = models::EvalLogMotionModel(odom_disp, odom_angle_disp,
                                                                  hypothesis_disp, hypothesis_rot);

        if (ignore_motion_model) {
          plausible_entries.emplace(index, 0);
        } else if (log_motion_prob > log_motion_prob_threshold) {
          plausible_entries.emplace(index, log_motion_prob);
        }
      }
    }
  }

  // Evaluate the observation likelihood model on plausible indices
  // according to the motion model.
  //
  // Since the only care about the index with the maximum likelihood,
  // we can keep track of the running maximium likelihood and prune
  // while evaluating the model if we know that the current likelihood
  // cannot be greater than the maximum likelihood.
  double max_prob = -std::numeric_limits<double>::infinity();
  double max_obs_prob = max_prob;

  while (!plausible_entries.empty()) {
    Entry e = plausible_entries.top();
    plausible_entries.pop();

    const Point& index = e.index;
    const double log_motion_prob = e.prob;

    const auto [d_loc, d_theta] = unbinify(index);
    const Eigen::Rotation2Df dtheta_rot(d_theta);
    double log_sum = 0;

    // CRITICAL PRUNING ASSUMPTION: ref_map.query (LogObsModel) is never positive
    const double prune_threshold = max_prob - log_motion_prob;
    bool prune = false;

    for (Eigen::Index scan_i = 0; scan_i < new_obs.point_cloud().cols(); scan_i += 10) {
      const Eigen::Vector2f& point = new_obs.point_cloud().col(scan_i);

      // Translate the observation into the previous reference frame.
      const Eigen::Vector2f query_point = dtheta_rot * point + d_loc;
      const float query_dist = query_point.norm();
      if (query_dist <= new_obs.min_range_dist_ || query_dist >= new_obs.max_range_dist_) {
        continue;
      }

      double log_obs_prob = ref_map.query(query_point.x(), query_point.y());
      // approximate the symmetric robust observation likelihood model
      log_obs_prob = std::max(log_obs_prob, log_obs_prob_threshold);
      log_sum += log_obs_prob;

      if (enable_obs_pruning && log_sum < prune_threshold) {
        prune = true;
        break;
      }
    }

    if (!prune) {
      double belief_prob = log_motion_prob + log_sum;
      max_prob = std::max(max_prob, belief_prob);
      max_obs_prob = std::max(max_obs_prob, log_sum);
      cube_.emplace(index, belief_prob);
    }
  }

  // We currently ignore the motion model for the coarse cube, so
  // return the observation likelihood probability for the fine cube.
  return max_obs_prob;
}

std::pair<Eigen::Vector2f, double> BeliefCube::max_belief() const {
  auto __delayedfn = common::runtime_dist.auto_lap("BeliefCube::max_belief");
  if (max_belief_.has_value()) {
    return max_belief_.value();
  }

  max_belief_ = unbinify(max_index_iterator()->first);
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
