#ifndef SRC_SLAM_RASTER_MAP_HH_
#define SRC_SLAM_RASTER_MAP_HH_

#include <unordered_map>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "util/matrix_hash.hh"

/**
 * TODO: doc
 *
 *
 *
 *
 * resolution: 0.04m x 0.04m square
 *
 */
class RasterMap {
  using Point = Eigen::Vector2i;

 public:
  RasterMap(const sensor_msgs::LaserScan& obs);

  int binify(double v) const;
  double unbinify(int i) const;

 private:
  static constexpr int resolution_ = 4;  // centimeters

 private:
  std::unordered_map<Point, double, util::EigenMatrixHash<Point>> raster_table_;
};

#endif  // SRC_SLAM_RASTER_MAP_HH_
