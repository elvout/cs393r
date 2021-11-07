#ifndef SRC_SLAM_RASTER_MAP_HH_
#define SRC_SLAM_RASTER_MAP_HH_

#include <unordered_map>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "util/matrix_hash.hh"

namespace slam {

/**
 * A rasterized lookup table of conditional probability values
 * from the observation likelihood model based on sensor observations
 * in an arbitrary time step.
 *
 * The resulting image is essentially a 2D histogram of the observation
 * space. The value of a bin is the maximum probability that the bin
 * could be observed given all sensor observations.
 *
 * The rasterized table is implemented as a dictionary of keys sparse
 * matrix. A dictionary-of-keys implementation should have the following
 * advantages over an image-matrix-based implementation when the table
 * is sparse:
 *  - efficient iteration over non-zero points
 *  - trivial storage of negative keys
 *  - preserves the coordinate space structure in the index space
 *  - efficient map transformation
 *
 * The map is indexed using integer centimeters since floating points
 * may have rounding errors.
 * Uses a bin resolution of 0.04m x 0.04m.
 *
 * TODO: merge(const RasterMap&) for parallel construction and combination
 * TODO: explicit inlining?
 * TODO: Eigen::Vector2f origin_point_ ?
 */
class RasterMap {
  using Point = Eigen::Vector2i;

 public:
  RasterMap(const int resolution);

  /// Generate the rasterized map given laser scan readings.
  void eval(const sensor_msgs::LaserScan& obs);

  /// Return the log probability value at the specified coordinate.
  double query(const double x, const double y) const;
  double query(const Eigen::Vector2f& coord) const;

  // Dump the contents of the raster table into a file.
  void dump_csv(const std::string filename = "image.csv") const;

 private:
  /// Convert a coordinate value in meters to a value in the index space.
  int meters_to_index(const double meters) const;
  Point binify(const double x, const double y) const;
  Point binify(const Eigen::Vector2f& coord) const;

  /// Convert an index value to a coordinate value in meters.
  double index_to_meters(const int index) const;
  Eigen::Vector2f unbinify(const Point& index) const;

 private:
  const int resolution_;  // centimeters
  std::unordered_map<Point, double, util::EigenMatrixHash<Point>> raster_table_;
};

}  // namespace slam

#endif  // SRC_SLAM_RASTER_MAP_HH_
