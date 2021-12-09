#include "rd_slam/rd_slam.hh"

#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <unordered_set>
#include <vector>
#include "common/common.hh"
#include "eigen3/Eigen/Dense"
#include "math/line2d.h"
#include "math/math_util.h"
#include "models/sensor.hh"
#include "rd_slam/iepf.hh"
#include "sensor_msgs/LaserScan.h"
#include "shared/math/line2d.h"

namespace {
template <class T>
using MinHeap = std::priority_queue<T, std::vector<T>, std::greater<T>>;

constexpr double f64INF = std::numeric_limits<double>::infinity();
}  // namespace

namespace rd_slam {

/**
 * See accompanying header docstring.
 */
double LSS(geometry::line2f a, geometry::line2f b, bool strict) {
  // Set `a` to the longer line segment.
  if (b.Length() > a.Length()) {
    std::swap(a, b);
  }

  // Transform the lines such that:
  //  - endpoint `p0` of `a` is the origin
  //  - `a` lies on the x-axis.
  //  - endpoint `p0` of `b` is closer to the origin
  Eigen::Affine2f xform = Eigen::Affine2f::Identity();
  xform = Eigen::Translation2f(-a.p0) * xform;
  a.p0 = xform * a.p0;
  a.p1 = xform * a.p1;

  xform = Eigen::Rotation2Df(-std::atan2(a.p1.y(), a.p1.x())) * xform;
  a.p1 = xform.rotation() * a.p1;

  b.p0 = xform * b.p0;
  b.p1 = xform * b.p1;

  if (b.p1.norm() < b.p0.norm()) {
    std::swap(b.p0, b.p1);
  }

  // Rotation similarity metric
  double rot_score = 0;
  {
    const Eigen::Vector2f vec = b.p1 - b.p0;
    const double theta = std::atan2(vec.y(), vec.x());
    rot_score = std::abs(std::sin(theta));

    rot_score *= a.Length() / b.Length();
  }

  // Perpendicular similarity metric
  double perp_score = std::abs((b.p0.y() + b.p1.y()) / 2.0);
  // if (strict && perp_score > b.Length() / 2.0) {
  //   return f64INF;
  // }

  // Parallel similarity metric
  double pll_score = 0;
  {
    const double len = a.p1.x();

    if (0 <= b.p0.x() && b.p0.x() <= len) {
      pll_score = 0;
    } else if (0 <= b.p1.x() && b.p1.x() <= len) {
      pll_score = 0;
    } else {
      const double p0_dist = -b.p0.x();
      const double p1_dist = b.p1.x() - len;
      pll_score = std::min(p0_dist, p1_dist);
    }
  }
  if (strict && pll_score != 0) {
    return f64INF;
  }

  return Eigen::Vector3d(rot_score, perp_score, pll_score).norm();
}

SLAM::SLAM() : belief_history_(), odom_mutex_(), odom_epoch_(0) {}

void SLAM::ObserveOdometry(const pose_2d::Pose2Df new_pose) {
  std::scoped_lock lock(odom_mutex_);

  odom_epoch_++;
  odom_reported_pose_ = new_pose;
}

void SLAM::ObserveLaser(const sensor_msgs::LaserScan& scan) {
  static uint64_t last_odom_epoch = 0;

  pose_2d::Pose2Df reference_pose;
  {
    std::scoped_lock lock(odom_mutex_);

    if (last_odom_epoch == odom_epoch_) {
      return;
    } else {
      last_odom_epoch = odom_epoch_;
      reference_pose = odom_reported_pose_;
    }
  }
  auto __scopedfn = common::runtime_dist.auto_lap("SLAM::ObserveLaser");

  const models::Observations obs(scan);
  std::vector<geometry::line2f> segments = iterative_end_point_fit(obs.point_cloud());

  if (belief_history_.empty()) {
    belief_history_.emplace_back(std::move(segments), reference_pose, pose_2d::Pose2Df(0, 0, 0));
  } else {
    using Correspondence = SLAMBelief::Correspondence;

    // Naive correspondence matching based on min LSS.
    // ~O(N^2 log(N)), but N is usually small.
    MinHeap<std::pair<double, Correspondence>> q;
    const std::vector<geometry::line2f>& prev_segments = belief_history_.back().segments_;
    const std::vector<uint64_t>& prev_ids = belief_history_.back().segment_ids_;
    const pose_2d::Pose2Df& prev_odom = belief_history_.back().ref_odom_;

    Eigen::Vector2f disp = Eigen::Rotation2Df(-prev_odom.angle) *
                           (reference_pose.translation - prev_odom.translation);
    float angle_disp = math_util::ReflexToConvexAngle(reference_pose.angle - prev_odom.angle);

    // if (!(disp.norm() > 0.1 || std::abs(angle_disp) > 0.0872664626)) {
    //   return;
    // }

    pose_2d::Pose2Df rel_disp(angle_disp, disp);

    for (size_t prev_i = 0; prev_i < prev_segments.size(); prev_i++) {
      for (size_t cur_i = 0; cur_i < segments.size(); cur_i++) {
        const double lss = LSS(prev_segments[prev_i], segments[cur_i]);

        if (lss != f64INF) {
          q.emplace(std::make_pair(lss, Correspondence(prev_i, cur_i)));
        }
      }
    }

    std::vector<uint64_t> segment_ids(segments.size(), SLAMBelief::INVALID_ID);
    std::vector<Correspondence> corrs;
    std::unordered_set<size_t> prev_used_idx;
    std::unordered_set<size_t> cur_used_idx;

    while (!q.empty()) {
      const auto& [lss, c] = q.top();

      if (prev_used_idx.count(c.prev_i) == 0 && cur_used_idx.count(c.cur_i) == 0) {
        prev_used_idx.insert(c.prev_i);
        cur_used_idx.insert(c.cur_i);
        segment_ids[c.cur_i] = prev_ids[c.prev_i];
        corrs.push_back(c);
      }

      q.pop();
    }

    for (uint64_t& id : segment_ids) {
      if (id == SLAMBelief::INVALID_ID) {
        id = SLAMBelief::gen_id();
      }
    }

    belief_history_.emplace_back(std::move(segments), std::move(segment_ids), std::move(corrs),
                                 reference_pose, rel_disp);
  }
}

}  // namespace rd_slam
