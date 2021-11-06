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
#include <vector>

#include "belief_cube.hh"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

// Components of the probabilistic model of the SLAM belief
// at an arbitrary timestep.
struct SLAMBelief {
  // for use in this time step
  Eigen::Vector2f odom_disp;
  float odom_angle_disp;
  Eigen::Vector2f ref_loc;
  float ref_angle;
  BeliefCube belief_lookup;
  sensor_msgs::LaserScan obs;

  // for use in the next time step
  RasterMap ref_map;

  std::vector<Eigen::Vector2f> correlated_points(const RasterMap& prev_ref_map);
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
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  void OfflineBelEvaluation();

 private:
  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  std::vector<SLAMBelief> belief_history;
  bool offline_eval_;
};
}  // namespace slam

#endif  // SRC_SLAM_H_
