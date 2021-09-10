#include "navigation/toc.h"

#include <cmath>
#include "eigen3/Eigen/Dense"
#include "navigation/navigation.h"

namespace navigation::toc {

Plan_1D::Plan_1D(const Eigen::Vector2f& start_loc, const float target_displacement)
    : start_loc_(start_loc), target_displacement_(target_displacement) {
  peak_speed_ = std::sqrt((2 * target_displacement_ * kMaxAccel * std::abs(kMaxDecel)) /
                          (kMaxAccel + std::abs(kMaxDecel)));

  // handle both trapezoid cases at the same time
  peak_speed_ = std::min(kMaxSpeed, peak_speed_);
  decel_disp_threshold_ =
      target_displacement_ - (peak_speed_ * peak_speed_) / (2 * std::abs(kMaxDecel));
}

}  // namespace navigation::toc
