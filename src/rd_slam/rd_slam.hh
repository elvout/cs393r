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

/**
 * Line Segment Similarity Metric
 *
 * Measures the similarity between two lines using distance metrics in subspaces
 * of SE(2).
 *
 * The `strict` flag is used to enforce hard constraints on the parallel and
 * perpendicular distance metrics. It should generally be set to true when
 * establishing correspondences and false when running optimization.
 *
 * Since the range of |sin| is [0, 1], the rotation distance metric is scaled by
 * the ratio of line segment lengths to discourage correspondences between
 * segments with very dissimilar lengths.
 */
double LSS(geometry::line2f a, geometry::line2f b, bool strict = true);

struct SLAMBelief {
  struct Correspondence {
    size_t prev_i;
    size_t cur_i;
    double lss;

    Correspondence(size_t p, size_t c, size_t l) : prev_i(p), cur_i(c), lss(l) {}

    bool operator>(const Correspondence& other) const { return lss > other.lss; }
  };

  SLAMBelief(std::vector<geometry::line2f>&& segments, const pose_2d::Pose2Df& ref_pose)
      : segments(segments), corrs(), ref_pose(ref_pose) {}
  SLAMBelief(std::vector<geometry::line2f>&& segments,
             std::vector<Correspondence>&& corrs,
             const pose_2d::Pose2Df& ref_pose)
      : segments(segments), corrs(corrs), ref_pose(ref_pose) {}

 public:
  std::vector<geometry::line2f> segments;
  std::vector<Correspondence> corrs;
  pose_2d::Pose2Df ref_pose;
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
