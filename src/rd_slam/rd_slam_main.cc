#include <vector>

#include "eigen3/Eigen/Dense"
#include "shared/math/line2d.h"

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization/visualization.h"

#include "gflags/gflags.h"

#include "models/sensor.hh"
#include "rd_slam/iepf.hh"

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

  models::Observations obs(msg);
  std::vector<geometry::line2f> lines = rd_slam::iterative_end_point_fit(obs.point_cloud());

  visualization::ClearVisualizationMsg(vis_msg_);
  vis_msg_.header.stamp = ros::Time::now();

  for (const geometry::line2f& line : lines) {
    visualization::DrawLine(line.p0, line.p1, 0x00a6ff, vis_msg_);
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

  return 0;
}
