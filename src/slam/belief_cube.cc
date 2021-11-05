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

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const sensor_msgs::LaserScan& new_obs) {
  const std::vector<Eigen::Vector2f> obs_points = PointsFromScan(new_obs);

  // motion model
  size_t evals = 0;
  // not a great threshold, as the standard devations are dependent on the magnitude
  // of displacement
  const double log_prob_threshold = LogNormalPdf(3, 0, 1);
  for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
    for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
      for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
        evals++;

        Eigen::Vector2f hypothesis_disp(dx / 100.0, dy / 100.0);  // meters
        double hypothesis_rot = math_util::DegToRad(static_cast<double>(dtheta));

        double log_motion_prob =
            LogMotionModel(odom_disp, odom_angle_disp, hypothesis_disp, hypothesis_rot);

        if (log_motion_prob > log_prob_threshold) {
          // TODO: bug-prone code: write safe wrapper and document
          Point key(dx / tx_resolution_, dy / tx_resolution_, dtheta);
          cube_[key] += log_motion_prob;
        }
      }
    }
  }

  printf("[BeliefCube::eval INFO] cube size: %lu / %lu\n", cube_.size(), evals);

  // observation likelihood model
  // 2D slicing was too slow, unless I was just doing it wrong
  size_t prune_count = 0;
  for (auto it = cube_.begin(); it != cube_.cend();) {
    const Point& index = it->first;

    auto [d_loc, d_theta] = unbinify(index);
    const Eigen::Rotation2Df dtheta_rot(d_theta);
    size_t hits = 0;
    double log_sum = 0;

    for (size_t scan_i = 0; scan_i < obs_points.size(); scan_i += 10) {
      const Eigen::Vector2f& point = obs_points[scan_i];

      const Eigen::Vector2f& query_point = dtheta_rot * point + d_loc;
      const float query_dist = query_point.norm();
      if (query_dist <= new_obs.range_min || query_dist >= new_obs.range_max) {
        continue;
      }

      double log_obs_prob = ref_map.query(query_point.x(), query_point.y());
      log_obs_prob = std::max(log_obs_prob, -2.0);  // TODO: un-hardcode, about 2.5 stddev
      if (log_obs_prob != -std::numeric_limits<double>::infinity()) {
        // it->second += log_obs_prob;
        log_sum += log_obs_prob;
        hits++;
      }

      // it->second += log_obs_prob;

      // if (log_obs_prob == -std::numeric_limits<double>::infinity()) {
      //   break;
      // }
    }

    // prune out impossible values to sparsify matrix
    // if (it->second == -std::numeric_limits<double>::infinity()) {
    // double min_hit_threshold = obs_points.size() / 10 / 3;
    double min_hit_threshold = 5;
    if (hits < min_hit_threshold) {
      it = cube_.erase(it);
      prune_count++;
    } else {
      it->second += log_sum * 0.12;
      it++;
    }
  }

  printf("[BeliefCube::eval INFO] negative inf prune count: %lu\n", prune_count);
  printf("[BeliefCube::eval INFO] remaining cube size: %lu\n", cube_.size());

  // observation likelihood model
  // for (const Eigen::Vector2f& point : obs_points) {
  // for (size_t i = 0; i < obs_points.size(); i += 10) {
  //   printf("debug: %lu\n", i);
  //   const Eigen::Vector2f& point = obs_points[i];

  //   for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
  //     // negative?
  //     const Eigen::Rotation2Df dtheta_rot(math_util::DegToRad(static_cast<double>(dtheta)));
  //     Eigen::Vector2f proj_point = dtheta_rot * point;

  //     for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
  //       const double query_x = proj_point.x() + (dx / 100.0);  // meters
  //       for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
  //         const double query_y = proj_point.y() + (dy / 100.0);  // meters

  //         double obs_prob = ref_map.query(query_x, query_y);
  //         Point key = binify(dx, dy, dtheta);
  //         cube_[key] += std::log(obs_prob);
  //       }
  //     }
  //   }
  // }

  // motion model
  // retain only non-zero points
  // for (auto it = cube_.begin(); it != cube_.cend();) {
  //   if (it->second == -std::numeric_limits<double>::infinity()) {
  //     it = cube_.erase(it);
  //   } else {
  //     it++;
  //   }
  // }

  // for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
  //   for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
  //     for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
  //       Point key(dx, dy, dtheta);
  //       auto it = cube_.find(key);
  //       if (it == cube_.cend()) {
  //         continue;
  //       }

  //       Eigen::Vector2f hypothesis_disp(dx / 100.0, dy / 100.0);  // meters
  //       double hypothesis_rot = math_util::DegToRad(static_cast<double>(dtheta));

  //       double motion_prob =
  //           MotionModel(odom_disp, odom_angle_disp, hypothesis_disp, hypothesis_rot);
  //       double log_prob = std::log(motion_prob);

  //       if (log_prob == -std::numeric_limits<double>::infinity()) {
  //         cube_.erase(it);
  //       } else {
  //         it->second += log_prob;
  //       }
  //     }
  //   }
  // }
}

std::pair<Eigen::Vector2f, double> BeliefCube::max_belief() const {
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

  return unbinify(max_index->first);
}

BeliefCube::Point BeliefCube::binify(const double x, const double y, const double rad) const {
  using math_util::ConstrainAngle;
  using math_util::RadToDeg;

  double x_bin = (x * 100.0 + tx_resolution_ * 0.5) / tx_resolution_;
  double y_bin = (y * 100.0 + tx_resolution_ * 0.5) / tx_resolution_;
  double theta_bin = RadToDeg(ConstrainAngle(rad));

  return Point(static_cast<int>(x_bin), static_cast<int>(y_bin), static_cast<int>(theta_bin));
}

BeliefCube::Point BeliefCube::binify(const Eigen::Vector2f& coord, const double rad) const {
  return binify(coord.x(), coord.y(), rad);
}

std::pair<Eigen::Vector2f, double> BeliefCube::unbinify(const Point& index) const {
  double x_coord = index.x() * tx_resolution_ / 100.0;
  double y_coord = index.y() * tx_resolution_ / 100.0;

  // potential for a bug:
  // in binify() we use single degree increments, but rot_resolution is not
  // necessarily a single degree.
  double rad = math_util::DegToRad(static_cast<double>(index.z()));

  return std::make_pair(Eigen::Vector2f(x_coord, y_coord), rad);
}
