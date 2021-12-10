//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <fstream>
#include <future>
#include <memory>
#include <vector>

#include "belief_cube.hh"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "models/sensor.hh"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

// Components of the probabilistic model of the SLAM belief
// at an arbitrary timestep.
struct SLAMBelief {
  SLAMBelief(const Eigen::Vector2f& prev_odom_loc,
             const float prev_odom_angle,
             const Eigen::Vector2f& odom_loc,
             const float odom_angle,
             const sensor_msgs::LaserScan& obs);

  // evidence for this time step
  Eigen::Vector2f odom_disp;
  float odom_angle_disp;
  models::Observations obs;

  // posterior belief of this time step
  Eigen::Vector2f belief_disp;
  float belief_angle_disp;
  Eigen::Vector2f belief_loc;
  float belief_angle;

  // evidence for the next time step
  std::shared_future<RasterMap> coarse_ref_map;
  std::shared_future<RasterMap> fine_ref_map;

  std::vector<Eigen::Vector2f> correlated_points(const RasterMap& prev_ref_map) const;
};

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const sensor_msgs::LaserScan& obs);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc, const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  std::pair<Eigen::Vector2f, float> GetPose() const;

  void EnableLogging();

 private:
  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  std::vector<std::shared_ptr<SLAMBelief>> belief_history;

  IdentityRasterMap map_;

  std::ofstream log_file_;
};
}  // namespace slam

#endif  // SRC_SLAM_H_
