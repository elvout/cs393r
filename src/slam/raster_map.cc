#include "raster_map.hh"
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <unordered_set>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "models/sensor.hh"
#include "util/matrix_hash.hh"

namespace {
// Manhattan direction array for flood-fill.
const std::array<Eigen::Vector2i, 4> dirs{
    Eigen::Vector2i(1, 0),
    Eigen::Vector2i(0, 1),
    Eigen::Vector2i(-1, 0),
    Eigen::Vector2i(0, -1),
};
}  // namespace

namespace slam {

RasterMap::RasterMap(const int resolution) : resolution_(resolution), raster_table_() {}

/**
 * Evaluates the probability of histogram bins around each observation
 * point until the probability falls below a threshold.
 */
void RasterMap::eval(const models::Observations& obs) {
  raster_table_.clear();

  // Keep track of which bins contain observation points. Assume
  // observations are of the same object if they are in the same bin.
  // As an optimization, skip expansions around subsequent observations
  // of the same object.
  std::unordered_set<Point, util::EigenMatrixHash<Point>> observation_bins;

  const double log_prob_threshold = models::RobustLogSensorModelThreshold(2.5);
  for (Eigen::Index col_i = 0; col_i < obs.point_cloud().cols(); col_i++) {
    const Eigen::Vector2f& observed_point = obs.point_cloud().col(col_i);
    const Point observed_bin = binify(observed_point);

    if (observation_bins.count(observed_bin) == 1) {
      continue;
    }
    observation_bins.insert(observed_bin);

    // depth-first flood fill expansion around the observed point
    std::vector<Point> remaining;
    std::unordered_set<Point, util::EigenMatrixHash<Point>> visited;
    remaining.push_back(observed_bin);
    visited.insert(observed_bin);

    while (!remaining.empty()) {
      const Point bin = remaining.back();
      remaining.pop_back();

      const Point bin_dist = bin - observed_bin;
      const Eigen::Vector2f coord_dist = unbinify(bin_dist);
      const Eigen::Vector2f bin_point = observed_point + coord_dist;
      const double log_prob = models::EvalLogSensorModel(observed_point, bin_point);

      if (log_prob > log_prob_threshold) {
        // This bin won't be revisited during this expansion, but the expansion
        // around a different observation point can visited this bin as well.
        auto bin_it = raster_table_.find(bin);
        if (bin_it == raster_table_.cend()) {
          raster_table_.emplace(bin, log_prob);
        } else if (log_prob > bin_it->second) {
          bin_it->second = log_prob;
        }

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

  printf("[RasterMap::eval INFO] table size: %lu\n", raster_table_.size());
}

/// Return the log probability value at the specified coordinate.
double RasterMap::query(const double x, const double y) const {
  const Point table_index = binify(x, y);

  auto it = raster_table_.find(table_index);
  if (it == raster_table_.cend()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return it->second;
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
 * The origin histogram bin is centered around [0, 0]. Thus, the
 * euclidean boundaries of the bin are [-resolution_ / 2, resolution / 2].
 */
int RasterMap::meters_to_index(const double meters) const {
  // TODO: breaks ties towards positive inf
  // It should probably be either towards or away from 0 instead
  // for positive/negative symmetry.

  const double cm = meters * 100.0;
  const double bin = (cm + resolution_ * 0.5) / resolution_;
  return static_cast<int>(bin);
}

RasterMap::Point RasterMap::binify(const double x, const double y) const {
  return Point(meters_to_index(x), meters_to_index(y));
}

RasterMap::Point RasterMap::binify(const Eigen::Vector2f& coord) const {
  return binify(coord.x(), coord.y());
}

/**
 * Convert an index-space value to a coordinate-space value in meters.
 *
 * The coordinate value of the center of the corresponding histogram bin
 * is returned.
 */
double RasterMap::index_to_meters(const int index) const {
  const double cm = index * resolution_;
  return cm / 100.0;
}

Eigen::Vector2f RasterMap::unbinify(const Point& index) const {
  return Eigen::Vector2f(index_to_meters(index.x()), index_to_meters(index.y()));
}

void RasterMap::dump_csv(const std::string filename) const {
  std::ofstream fout(filename);

  for (const auto& [index, prob] : raster_table_) {
    fout << index.x() << ',' << index.y() << ',' << prob << '\n';
  }
}

IdentityRasterMap::IdentityRasterMap(const int resolution) : RasterMap(resolution) {}

void IdentityRasterMap::add_coord(const Eigen::Vector2f& coord) {
  raster_table_.emplace(binify(coord), 0);
}

std::vector<Eigen::Vector2f> IdentityRasterMap::export_coord_map() const {
  std::vector<Eigen::Vector2f> coords;

  for (const auto& [index, _prob] : raster_table_) {
    coords.push_back(unbinify(index));
  }

  return coords;
}

}  // namespace slam
