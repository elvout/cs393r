#include "raster_map.hh"
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/LaserScan.h"

namespace {
const Eigen::Vector2f laser_loc(0.2, 0);
}

RasterMap::RasterMap(const sensor_msgs::LaserScan& obs) : raster_table_() {}

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
