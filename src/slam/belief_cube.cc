#include "belief_cube.hh"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math/math_util.h"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);

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
    points.push_back(laser_loc + scan_rot * Eigen::Vector2f(scan_range, 0));
  }

  return points;
}

double LogNormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for log(1 / sqrt(2pi))
  constexpr double log_inv_sqrt2pi = -0.91893853320467274178;

  const double z = (val - mean) / stddev;
  return log_inv_sqrt2pi - std::log(stddev) - (z * z * 0.5);
}

double MotionModel(const Eigen::Vector2f& expected_disp,
                   const double expected_rot,
                   const Eigen::Vector2f& hypothesis_disp,
                   const double hypothesis_rot) {
  // TODO: move params to config?
  constexpr double k1 = 0.45;
  constexpr double k2 = 1.6;
  constexpr double k3 = 0.65;
  constexpr double k4 = 2.3;

  const double expected_disp_n = expected_disp.norm();
  const double expected_rot_n = std::abs(math_util::ReflexToConvexAngle(expected_rot));
  const double disp_std = k1 * expected_disp_n + k2 * expected_rot_n;
  const double rot_std = k3 * expected_disp_n + k4 * expected_rot_n;

  const double x_noise = hypothesis_disp.x() - expected_disp.x();
  const double y_noise = hypothesis_disp.y() - expected_disp.y();
  const double theta_noise = math_util::ReflexToConvexAngle(hypothesis_rot - expected_rot);

  const double log_px = LogNormalPdf(x_noise, 0, disp_std);
  const double log_py = LogNormalPdf(y_noise, 0, disp_std);
  const double log_ptheta = LogNormalPdf(theta_noise, 0, rot_std);

  return log_px + log_py + log_ptheta;
}

}  // namespace

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const sensor_msgs::LaserScan& new_obs) {
  const std::vector<Eigen::Vector2f> obs_points = PointsFromScan(new_obs);

  // motion model
  size_t evals = 0;
  for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
    for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
      for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
        evals++;

        Eigen::Vector2f hypothesis_disp(dx / 100.0, dy / 100.0);  // meters
        double hypothesis_rot = math_util::DegToRad(static_cast<double>(dtheta));

        double log_motion_prob =
            MotionModel(odom_disp, odom_angle_disp, hypothesis_disp, hypothesis_rot);

        // e^-10 is approx 4e-5
        if (log_motion_prob > -10) {
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
    const Eigen::Rotation2Df dtheta_rot(-d_theta);
    for (size_t scan_i = 0; scan_i < obs_points.size(); scan_i += 10) {
      const Eigen::Vector2f& point = obs_points[scan_i];

      const Eigen::Vector2f& query_point = dtheta_rot * point + d_loc;

      double obs_prob = ref_map.query(query_point.x(), query_point.y());
      double log_prob = std::log(obs_prob);
      it->second += log_prob;

      if (log_prob == -std::numeric_limits<double>::infinity()) {
        break;
      }
    }

    // prune out impossible values to sparsify matrix
    if (it->second == -std::numeric_limits<double>::infinity()) {
      it = cube_.erase(it);
      prune_count++;
    } else {
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
