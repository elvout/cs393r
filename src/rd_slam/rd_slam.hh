#ifndef SRC_RD_SLAM_RD_SLAM_HH_
#define SRC_RD_SLAM_RD_SLAM_HH_

#include <mutex>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math/line2d.h"
#include "sensor_msgs/LaserScan.h"
#include "shared/math/line2d.h"
#include "shared/math/poses_2d.h"

namespace rd_slam {

class SLAMBelief {
 public:
  struct Correspondence {
    size_t prev_i;
    size_t cur_i;

    Correspondence(size_t p, size_t c) : prev_i(p), cur_i(c) {}

    bool operator>(const Correspondence& other) const { return prev_i > other.prev_i; }
    bool operator<(const Correspondence& other) const { return prev_i < other.prev_i; }
  };

  static constexpr uint64_t INVALID_ID = 0;
  static uint64_t gen_id() {
    static uint64_t next_id = INVALID_ID + 1;
    return next_id++;
  }

 public:
  SLAMBelief(std::vector<geometry::line2f>&& segments,
             const pose_2d::Pose2Df& ref_odom,
             const pose_2d::Pose2Df& rel_disp)
      : segments_(segments), segment_ids_(), corrs_(), ref_odom_(ref_odom), rel_disp_(rel_disp) {
    segment_ids_.reserve(segments_.size());
    for (size_t i = 0; i < segments_.size(); i++) {
      segment_ids_.push_back(gen_id());
    }
  }

  SLAMBelief(std::vector<geometry::line2f>&& segments,
             std::vector<uint64_t>&& segment_ids,
             std::vector<Correspondence>&& corrs,
             const pose_2d::Pose2Df& ref_odom,
             const pose_2d::Pose2Df& rel_disp)
      : segments_(segments),
        segment_ids_(segment_ids),
        corrs_(corrs),
        ref_odom_(ref_odom),
        rel_disp_(rel_disp) {}

 public:
  std::vector<geometry::line2f> segments_;
  std::vector<uint64_t> segment_ids_;
  std::vector<Correspondence> corrs_;
  pose_2d::Pose2Df ref_odom_;
  pose_2d::Pose2Df rel_disp_;
};

class SLAM {
 public:
  SLAM();

  void ObserveLaser(const sensor_msgs::LaserScan& scan);
  void ObserveOdometry(const pose_2d::Pose2Df new_pose);

 public:
  std::vector<SLAMBelief> belief_history_;

  // Mutual exclusion probably isn't necessary, but it may prevent a
  // one-in-a-zillion read-before-write bug.
  std::mutex odom_mutex_;
  uint64_t odom_epoch_;  // 0 signifies odometry has not been initialized
  pose_2d::Pose2Df odom_reported_pose_;
};

}  // namespace rd_slam

#endif  // SRC_RD_SLAM_RD_SLAM_HH_
