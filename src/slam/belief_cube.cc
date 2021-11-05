#include "belief_cube.hh"
#include <iostream>
#include <limits>
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

// TODO: refactor, move to shared or util library
double NormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2pi))
  constexpr double inv_sqrt2pi = 0.39894228040143267794;

  double z = (val - mean) / stddev;
  return inv_sqrt2pi / stddev * std::exp(-0.5 * z * z);
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

  const double px = NormalPdf(x_noise, 0, disp_std);
  const double py = NormalPdf(y_noise, 0, disp_std);
  const double ptheta = NormalPdf(theta_noise, 0, rot_std);

  // todo: log-likelihoods?
  return px * py * ptheta;
}

}  // namespace

// in raster_map we compute the map in the constructor
// should we make the APIs consistent?
BeliefCube::BeliefCube() : cube_() {}

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const sensor_msgs::LaserScan& new_obs) {
  const std::vector<Eigen::Vector2f> obs_points = PointsFromScan(new_obs);

  // observation likelihood model
  for (const Eigen::Vector2f& point : obs_points) {
    for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
      // negative?
      const Eigen::Rotation2Df dtheta_rot(math_util::DegToRad(static_cast<double>(dtheta)));
      Eigen::Vector2f proj_point = dtheta_rot * point;

      for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
        const double query_x = proj_point.x() + (dx / 100.0);  // meters
        for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
          const double query_y = proj_point.y() + (dy / 100.0);  // meters

          double obs_prob = ref_map.query(query_x, query_y);
          Point key = binify(dx, dy, dtheta);
          cube_[key] += std::log(obs_prob);
        }
      }
    }
  }

  // motion model
  // iterate over non-zero points
  for (auto it = cube_.begin(); it != cube_.cend();) {
    if (it->second == -std::numeric_limits<double>::infinity()) {
      it = cube_.erase(it);
    } else {
      it++;
    }
  }

  // TODO: can iterate over non-zero cells directly
  for (int dtheta = 0; dtheta <= rot_windowsize_; dtheta += rot_resolution_) {
    for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
      for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
        Point key(dx, dy, dtheta);
        auto it = cube_.find(key);
        if (it == cube_.cend()) {
          continue;
        }

        Eigen::Vector2f hypothesis_disp(dx / 100.0, dy / 100.0);  // meters
        double hypothesis_rot = math_util::DegToRad(static_cast<double>(dtheta));

        double motion_prob =
            MotionModel(odom_disp, odom_angle_disp, hypothesis_disp, hypothesis_rot);
        double log_prob = std::log(motion_prob);

        if (log_prob == -std::numeric_limits<double>::infinity()) {
          cube_.erase(it);
        } else {
          it->second += log_prob;
        }
      }
    }
  }
}

std::pair<Eigen::Vector2f, double> BeliefCube::max_belief() const {
  if (cube_.empty()) {
    std::cerr << "[BeliefCube::max_belief() FATAL]: empty cube\n";
    exit(1);
  }

  auto it = cube_.begin();
  auto max_index = it;

  while (it++ != cube_.cend()) {
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
  double x_coord = index.x() * tx_resolution_ * 100.0;
  double y_coord = index.y() * tx_resolution_ * 100.0;

  // potential for a bug:
  // in binify() we use single degree increments, but rot_resolution is not
  // necessarily a single degree.
  double rad = math_util::DegToRad(static_cast<double>(index.z()));

  return std::make_pair(Eigen::Vector2f(x_coord, y_coord), rad);
}
