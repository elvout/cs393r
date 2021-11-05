#include "raster_map.hh"
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "util/matrix_hash.hh"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);

// Inflated standard deviation value.
// Less inflation than the Particle filter to keep computation time down.
constexpr double CONFIG_LidarStddev = 0.08;

// Euclidean direction array for flood-fill/search.
const std::array<Eigen::Vector2i, 4> dirs{
    Eigen::Vector2i(1, 0),
    Eigen::Vector2i(0, 1),
    Eigen::Vector2i(-1, 0),
    Eigen::Vector2i(0, -1),
};

// TODO: refactor, move to shared or util library
double LogNormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for log(1 / sqrt(2pi))
  constexpr double log_inv_sqrt2pi = -0.91893853320467274178;

  const double z = (val - mean) / stddev;
  return log_inv_sqrt2pi - std::log(stddev) - (z * z * 0.5);
}

/**
 * A simple observation likelihood model using a Gaussian distribution.
 */
double ObsLikelihoodModel(const sensor_msgs::LaserScan& obs,
                          const Eigen::Vector2f& expected,
                          const Eigen::Vector2f& hypothesis) {
  float hypothesis_range = (hypothesis - laser_loc).norm();
  if (hypothesis_range <= obs.range_min || hypothesis_range >= obs.range_max) {
    return 0;
  }

  float range_diff = (expected - hypothesis).norm();
  return LogNormalPdf(range_diff, 0, CONFIG_LidarStddev);
}

}  // namespace

/**
 * Evaluates the probability of histogram bins around each observation
 * point until the probability falls below a threshold.
 */
void RasterMap::eval(const sensor_msgs::LaserScan& obs) {
  const std::vector<float>& ranges = obs.ranges;

  std::unordered_set<Point, util::EigenMatrixHash<Point>> observed_bins;

  for (size_t i = 0; i < ranges.size(); i++) {
    const float scan_range = ranges[i];
    if (scan_range <= obs.range_min || scan_range >= obs.range_max) {
      continue;
    }

    const float scan_angle = obs.angle_min + i * obs.angle_increment;
    const Eigen::Rotation2Df scan_rot(scan_angle);
    const Eigen::Vector2f observed_point = laser_loc + scan_rot * Eigen::Vector2f(scan_range, 0);
    const Point observed_bin(binify(observed_point.x()), binify(observed_point.y()));

    // assume readings are of the same object if they are in the same bin
    if (observed_bins.count(observed_bin) == 1) {
      continue;
    }
    observed_bins.insert(observed_bin);

    // dfs expansion around the observed point
    std::vector<Point> remaining;
    std::unordered_set<Point, util::EigenMatrixHash<Point>> visited;
    remaining.push_back(observed_bin);
    visited.insert(observed_bin);

    constexpr double log_prob_threshold = -20;  // about 6.17 standard deviations
    while (!remaining.empty()) {
      const Point bin = remaining.back();
      remaining.pop_back();

      const Point bin_dist = bin - observed_bin;
      const Eigen::Vector2f point_dist =
          Eigen::Vector2f(unbinify(bin_dist.x()), unbinify(bin_dist.y()));
      const Eigen::Vector2f bin_point = point_dist + observed_point;
      double prob = ObsLikelihoodModel(obs, observed_point, bin_point);

      if (prob > log_prob_threshold) {
        // This bin won't get revisited during this expansion, but the expansion
        // around another observation point could visit this bin as well.
        raster_table_[bin] = std::max(raster_table_[bin], prob);

        // add more stuff to the remaining queue
        for (const Point& dir : dirs) {
          const Point next_bin = bin + dir;
          if (visited.count(next_bin) == 0) {
            remaining.push_back(next_bin);
            visited.insert(next_bin);
          }
        }
      }
    }
  }
}

/// Return the log probability value at the specified coordinate.
double RasterMap::query(const double x, const double y) const {
  const Point table_index(binify(x), binify(y));

  auto element = raster_table_.find(table_index);
  if (element == raster_table_.cend()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return element->second;
  }
}

/// Return the log probability value at the specified coordinate.
double RasterMap::query(const Eigen::Vector2f& coord) const {
  return query(coord.x(), coord.y());
}

/**
 * Convert a coordinate space value specified in meters to a value in
 * the index space.
 *
 * Each origin histogram bin is centered around [0, 0]. Thus, the
 * euclidean boundaries of the bin are [-resolution_ / 2, resolution / 2].
 */
int RasterMap::binify(double v) const {
  // TODO: breaks ties towards positive inf
  // It should probably be either towards or away from 0 instead
  // for positive/negative symmetry.

  // convert to centimeters
  double cm = v * 100.0;
  double bin = (cm + resolution_ * 0.5) / resolution_;

  return static_cast<int>(bin);
}

/**
 * Convert an index-space value to a coordinate-space value in meters.
 *
 * The coordinate value of the center of the corresponding histogram bin
 * is returned.
 */
double RasterMap::unbinify(int bin) const {
  double cm = bin * resolution_;

  // convert to meters
  return cm / 100.0;
}
