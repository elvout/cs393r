#ifndef SRC_SLAM_RASTER_MAP_HH_
#define SRC_SLAM_RASTER_MAP_HH_

#include <unordered_map>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "util/matrix_hash.hh"

/**
 * A rasterized lookup table of conditional probability values
 * from the observation likelihood model based on sensor observations
 * in an arbitrary time step.
 *
 * The resulting image is essentially a 2D histogram of the observation
 * space. The value of a bin is the maximum probability that the bin
 * could be observed given all sensor observations.
 *
 * The rasterized table is implemented as a dictionary of keys since
 * the matrix is sparse. A dictionary-of-keys implementaiton has the
 * following advantages over a image-matrix-based implementation:
 *  - more efficient iteration over non-zero points
 *  - trivial storage of negative keys
 *  - preserves the coordinate space structure in the index space
 *
 * Uses a bin resolution of 0.04m x 0.04m.
 */
class RasterMap {
  using Point = Eigen::Vector2i;

 public:
  /// Construct a RasterMap with the given laser scan readings.
  RasterMap(const sensor_msgs::LaserScan& obs);

  /// Convert a coordinate value specified in meters to value in the index space.
  int binify(double v) const;

  /// Convert a pixel value to a coordinate value in meters.
  double unbinify(int i) const;

 private:
  static constexpr int resolution_ = 4;  // centimeters

 private:
  std::unordered_map<Point, double, util::EigenMatrixHash<Point>> raster_table_;
};

#endif  // SRC_SLAM_RASTER_MAP_HH_
