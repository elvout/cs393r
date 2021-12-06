#ifndef SRC_RD_SLAM_BACKEND_HH_
#define SRC_RD_SLAM_BACKEND_HH_

#include <stdexcept>
#include "ceres/ceres.h"
#include "math/line2d.h"
#include "rd_slam/rd_slam.hh"

namespace rd_slam {

class CorrespondenceConstraint {
 public:
  CorrespondenceConstraint(size_t a_idx,
                           size_t b_idx,
                           const geometry::line2f& a_line,
                           const geometry::line2f& b_line)
      : a_idx_(a_idx), b_idx_(b_idx), a_line_(a_line), b_line_(b_line) {
    if (b_idx_ < a_idx_) {
      throw std::runtime_error("[CorrespondenceConstraint]: invalid pose ordering");
    }
  }

  // `relative_poses` is an Nx3 matrix that stores the translation
  // and rotation of a relative pose transformation [x, y, theta]
  // [i] stores the transformation from Pose(i-1) to Pose(i)
  //
  // TODO: add constness to relative_poses, worry about this later
  template <typename T>
  bool operator()(T** relative_poses, T* residuals) const {
    // compose transformations from pose A to pose B
    Eigen::Affine2f xform = Eigen::Affine2f::Identity();

    for (size_t i = a_idx_ + 1; i <= b_idx_; i++) {
      xform = Eigen::Translation2f(relative_poses[i][0], relative_poses[i][1]) *
              Eigen::Rotation2Df(relative_poses[i][2]) * xform;
    }

    // Transform line A to the reference frame of pose B
    geometry::line2f a_line_in_b = a_line_;
    a_line_in_b.p0 = xform * a_line_in_b.p0;
    a_line_in_b.p1 = xform * a_line_in_b.p1;

    // compute the squared LSS between the two lines in pose B
    double lss = LSS(a_line_in_b, b_line_, false);
    lss = lss * lss;
    residuals[0] = lss;

    return true;
  }

 private:
  const size_t a_idx_;
  const size_t b_idx_;
  const geometry::line2f a_line_;
  const geometry::line2f b_line_;
};

}  // namespace rd_slam

#endif  // SRC_RD_SLAM_BACKEND_HH_
