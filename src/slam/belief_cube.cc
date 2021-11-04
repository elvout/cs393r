#include "belief_cube.hh"
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math/math_util.h"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);

std::vector<Eigen::Vector2f> points_from_scan(const sensor_msgs::LaserScan& scan) {
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
}  // namespace

// in raster_map we compute the map in the constructor
// should we make the APIs consistent?
BeliefCube::BeliefCube() : cube_() {}

void BeliefCube::eval(const RasterMap& ref_map, const sensor_msgs::LaserScan& new_obs) {
  const std::vector<Eigen::Vector2f> obs_points = points_from_scan(new_obs);

  for (int dtheta = 0; dtheta < rot_windowsize_; dtheta += rot_resolution_) {
    std::vector<Eigen::Vector2f> slice_points = obs_points;
    if (dtheta > 0) {
      const Eigen::Rotation2Df dtheta_rot(math_util::DegToRad(static_cast<double>(dtheta)));
      for (Eigen::Vector2f& point : slice_points) {
        point = dtheta_rot * point;
      }
    }

    for (const Eigen::Vector2f& point : slice_points) {
      for (int dx = -tx_windowsize_; dx <= tx_windowsize_; dx += tx_resolution_) {
        const double query_x = point.x() + (dx / 100.0);
        for (int dy = -tx_windowsize_; dy <= tx_windowsize_; dy += tx_resolution_) {
          const double query_y = point.y() + (dy / 100.0);

          double obs_prob = ref_map.query(query_x, query_y);

          // TODO: motion model
        }
      }
    }
  }
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
