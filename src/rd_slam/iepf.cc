#include "rd_slam/iepf.hh"

#include <deque>
#include <stdexcept>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "models/constraints.hh"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"

namespace {

/**
 * The start and end column indices (both inclusive) of a line segment in the
 * corresponding point cloud matrix.
 */
struct Interval {
  Eigen::Index start;
  Eigen::Index end;

  Interval(Eigen::Index start, Eigen::Index end) : start(start), end(end) {
    if (start > end) {
      throw std::runtime_error("[iepf Interval]: invalid endpoints");
    }
  }
};

constexpr float kOutlierThreshold = 0.1;
constexpr Eigen::Index kMinPointsPerLine = 9;
constexpr float kMinSegmentLength = 0.2;
constexpr float kMaxSuccPointDist = 1.0;

}  // namespace

namespace rd_slam {

/**
 * See the accompanying header docstring.
 *
 * IEPF is implemented in multiple steps.
 *
 * In the preprocessing step, the initial interval [0, N) is divided such that
 * the maximum distance between successive points in an interval does not exceed
 * a threshold.
 *
 * In the recursive segmentation step, intervals are divided using the point
 * furthest from the line segment formed by the interval as a split point. The
 * point-to-line-segment distance is calculated by treating the three points
 * (endpoints of the line segment and candidate split point) as a triangle with
 * the line segment as its base. The cross product of the vectors formed by
 * these points is the area of the corresponding parallelogram. The height of
 * the triangle then gives us the point-to-line-segment distance.
 */
std::vector<geometry::line2f> iterative_end_point_fit(
    const Eigen::Matrix<float, 2, Eigen::Dynamic>& point_cloud) {
  std::deque<Interval> segments;
  std::vector<geometry::line2f> lines;

  // Preprocessing
  for (Eigen::Index i = 0; i < point_cloud.cols() - 1; i++) {
    const Eigen::Vector2f& cur = point_cloud.col(i);
    const Eigen::Vector2f& next = point_cloud.col(i + 1);

    if ((next - cur).norm() > kMaxSuccPointDist) {
      if (segments.empty()) {
        segments.emplace_back(0, i);
      } else {
        segments.emplace_back(segments.back().end + 1, i);
      }
    }
  }
  if (!segments.empty()) {
    segments.emplace_back(segments.back().end + 1, point_cloud.cols() - 1);
  } else {
    segments.emplace_back(0, point_cloud.cols() - 1);
  }

  // Recursive Segmentation
  while (!segments.empty()) {
    const Interval cur_iv = segments.front();
    segments.pop_front();

    if (cur_iv.end - cur_iv.start + 1 < kMinPointsPerLine) {
      continue;
    }

    const Eigen::Vector2f& p0 = point_cloud.col(cur_iv.start);
    const Eigen::Vector2f& p1 = point_cloud.col(cur_iv.end);

    const Eigen::Vector2f v_base = p1 - p0;
    const float base_len = v_base.norm();

    Eigen::Index furthest_idx = cur_iv.start;
    float furthest_dist = 0;

    for (Eigen::Index i = cur_iv.start + 1; i < cur_iv.end; i++) {
      const Eigen::Vector2f& p2 = point_cloud.col(i);

      const Eigen::Vector2f v_side = p2 - p0;

      const float _2A = std::abs(geometry::Cross(v_base, v_side));
      const float point_dist = _2A / base_len;

      if (point_dist > furthest_dist) {
        furthest_dist = point_dist;
        furthest_idx = i;
      }
    }

    if (furthest_dist > kOutlierThreshold) {
      // FIFO
      segments.emplace_front(furthest_idx, cur_iv.end);
      segments.emplace_front(cur_iv.start, furthest_idx);
    } else if ((p1 - p0).norm() > kMinSegmentLength) {
      lines.emplace_back(p0, p1);
    }
  }

  return lines;
}

}  // namespace rd_slam
