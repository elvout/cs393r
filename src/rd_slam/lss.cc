#include "rd_slam/lss.hh"

#include <cmath>
#include "eigen3/Eigen/Dense"
#include "math/line2d.h"

namespace {
constexpr double f64INF = std::numeric_limits<double>::infinity();
}

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
    rot_score = std::abs(std::sin(theta)) * 3;

    rot_score *= a.Length() / b.Length();
  }

  // Perpendicular similarity metric
  double perp_score = std::abs((b.p0.y() + b.p1.y()) / 2.0);
  // if (strict && perp_score > b.Length() / 2.0) {
  //   return f64INF;
  // }

  // Parallel similarity metric
  double pll_score = 0;
  if (strict) {
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
}  // namespace rd_slam
