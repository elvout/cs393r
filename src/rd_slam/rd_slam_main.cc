#include <vector>

#include "eigen3/Eigen/Dense"
#include "shared/math/line2d.h"

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization/visualization.h"

#include "gflags/gflags.h"

#include "common/common.hh"
#include "models/sensor.hh"
#include "rd_slam/iepf.hh"
#include "rd_slam/rd_slam.hh"

using amrl_msgs::VisualizationMsg;

// Create command line arguments
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");
DECLARE_int32(v);

namespace {
ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
VisualizationMsg vis_msg_;
sensor_msgs::LaserScan last_laser_msg_;
rd_slam::SLAM slam_;
}  // namespace

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

  vis_msg_ = visualization::NewVisualizationMessage("map", "slam");
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  last_laser_msg_ = msg;

  common::runtime_dist.start_lap("ObserveLaser");
  slam_.ObserveLaser(msg);
  common::runtime_dist.end_lap("ObserveLaser");

  visualization::ClearVisualizationMsg(vis_msg_);
  vis_msg_.header.stamp = ros::Time::now();

  static std::vector<uint32_t> colors = {0xff0000, 0x00ff00, 0x0080ff,
                                         0x7F00FF, 0xFF00FF, 0x404040};
  size_t color = 0;

  if (slam_.belief_history_.size() > 1) {
    auto it = slam_.belief_history_.end();
    const rd_slam::SLAMBelief& cur = *(--it);
    const rd_slam::SLAMBelief& prev = *(--it);

    for (const auto& corr : cur.corrs) {
      const auto& prev_line = prev.segments[corr.prev_i];
      const auto& cur_line = cur.segments[corr.cur_i];

      visualization::DrawLine(prev_line.p0, prev_line.p1, colors[color], vis_msg_);
      visualization::DrawLine(cur_line.p0, cur_line.p1, colors[color], vis_msg_);

      color++;
      if (color == colors.size()) {
        color = 0;
      }
    }
  }

  visualization_publisher_.publish(vis_msg_);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Initialize ROS.
  ros::init(argc, argv, "rd_slam");
  ros::NodeHandle n;
  InitializeMsgs();

  visualization_publisher_ = n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ = n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);

  ros::Subscriber laser_sub = n.subscribe(FLAGS_laser_topic.c_str(), 1, LaserCallback);
  //   ros::Subscriber odom_sub = n.subscribe(FLAGS_odom_topic.c_str(), 1, OdometryCallback);

  ros::spin();

  printf("%s\n", common::runtime_dist.summary().c_str());

  return 0;
}