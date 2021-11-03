#include "raster_map.hh"
#include <array>
#include <cmath>
#include <unordered_set>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "util/matrix_hash.hh"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);

// TODO: refactor, move to shared or util library
double NormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2pi))
  constexpr double inv_sqrt2pi = 0.39894228040143267794;

  double z = (val - mean) / stddev;
  return inv_sqrt2pi / stddev * std::exp(-0.5 * z * z);
}

double NormalCdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2))
  constexpr double inv_sqrt2 = 0.70710678118654752440;

  double z = (val - mean) / stddev;
  return 0.5 + 0.5 * std::erf(z * inv_sqrt2);
}

double ObsLikelihoodModel(const sensor_msgs::LaserScan& obs,
                          const float expected,
                          const float hypothesis) {
  // Inflated standard deviation value.
  // Less inflation than the Particle filter to keep computation time down.
  constexpr double CONFIG_LidarStddev = 0.08;

  if (hypothesis <= obs.range_min || hypothesis >= obs.range_max) {
    return 0;
  }
  return NormalPdf(hypothesis, expected, CONFIG_LidarStddev);
}

/*
double ParticleFilterObsLikelihoodModel(const sensor_msgs::LaserScan& obs,
                                 const float expected,
                                 const float hypothesis) {
  // TODO: move to config reader
  constexpr double CONFIG_LidarStddev = 0.1;
  constexpr double CONFIG_GaussianLowerBound = -2.5 * CONFIG_LidarStddev;
  constexpr double CONFIG_GaussianUpperBound = 2.5 * CONFIG_LidarStddev;

  // approximation of the PDF integral, bad when hypothesis is around limits
  double p_integral = 0;
  // Integral for the lower interval.
  p_integral += (hypothesis + CONFIG_GaussianLowerBound - obs.range_min) *
                NormalPdf(CONFIG_GaussianLowerBound, 0, CONFIG_LidarStddev);
  // Integral for the Gaussian interval.
  p_integral += NormalCdf(CONFIG_GaussianUpperBound, 0, CONFIG_LidarStddev) -
                NormalCdf(CONFIG_GaussianLowerBound, 0, CONFIG_LidarStddev);
  // Integral for the upper interval.
  p_integral += (obs.range_max - (hypothesis + CONFIG_GaussianUpperBound)) *
                NormalPdf(CONFIG_GaussianUpperBound, 0, CONFIG_LidarStddev);

  double p = 0;
  if (hypothesis <= obs.range_min || hypothesis >= obs.range_max) {
    p = 0;
  } else if (hypothesis < expected + CONFIG_GaussianLowerBound) {
    p = NormalPdf(CONFIG_GaussianLowerBound, 0, CONFIG_LidarStddev);
  } else if (hypothesis > expected + CONFIG_GaussianUpperBound) {
    p = NormalPdf(CONFIG_GaussianUpperBound, 0, CONFIG_LidarStddev);
  } else {
    p = NormalPdf(hypothesis, expected, CONFIG_LidarStddev);
  }

  return p / p_integral;
}
*/

const std::array<Eigen::Vector2i, 4> dirs{
    Eigen::Vector2i(1, 0),
    Eigen::Vector2i(0, 1),
    Eigen::Vector2i(-1, 0),
    Eigen::Vector2i(0, -1),
};

}  // namespace

RasterMap::RasterMap(const sensor_msgs::LaserScan& obs) : raster_table_() {
  const std::vector<float>& ranges = obs.ranges;

  for (size_t i = 0; i < ranges.size(); i++) {
    const float scan_range = ranges[i];
    if (scan_range <= obs.range_min || scan_range >= obs.range_max) {
      continue;
    }

    const float scan_angle = obs.angle_min + i * obs.angle_increment;
    const Eigen::Rotation2Df scan_rot(scan_angle);
    const Eigen::Vector2f observed_point = laser_loc + scan_rot * Eigen::Vector2f(scan_range, 0);
    const Point observed_bin(binify(observed_point.x()), binify(observed_point.y()));

    // dfs expansion around the observed point
    std::vector<Point> remaining;
    std::unordered_set<Point, util::EigenMatrixHash<Point>> visited;
    remaining.push_back(observed_bin);
    visited.insert(observed_bin);

    constexpr double prob_threshold = 1e-5;  // about 4.6 standard deviations
    while (!remaining.empty()) {
      Point bin = remaining.back();
      remaining.pop_back();

      Point bin_dist = bin - observed_bin;
      Eigen::Vector2f bin_point =
          Eigen::Vector2f(unbinify(bin_dist.x()), unbinify(bin_dist.y())) + observed_point;
      float bin_range = (bin_point - laser_loc).norm();
      double prob = ObsLikelihoodModel(obs, scan_range, bin_range);

      if (prob > prob_threshold) {
        raster_table_[bin] += std::log(prob);

        // add more stuff to the remaining queue
        for (const Point& dir : dirs) {
          Point next_bin = bin + dir;
          if (visited.count(next_bin) == 0) {
            remaining.push_back(next_bin);
            visited.insert(next_bin);
          }
        }
      }
    }
  }
}

int RasterMap::binify(double v) const {
  // TODO: breaks ties towards positive inf
  // It should probably be either towards or away from 0 instead
  // for positive/negative symmetry.

  // convert to centimeters
  double cm = v * 100.0;
  double bin = (cm + resolution_ * 0.5) / resolution_;

  return static_cast<int>(bin);
}

double RasterMap::unbinify(int bin) const {
  double cm = bin * resolution_;

  // convert to meters
  return cm / 100.0;
}
