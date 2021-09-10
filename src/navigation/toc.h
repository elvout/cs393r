#ifndef CS393R_NAVIGATION_TOC_H_
#define CS393R_NAVIGATION_TOC_H_

#include "eigen3/Eigen/Dense"

namespace navigation::toc {

class Plan_1D {
 public:
  Plan_1D() = default;

  Plan_1D(const Eigen::Vector2f& start_loc, const float target_displacement);

  inline float target_displacement() const { return target_displacement_; }
  inline float peak_speed() const { return peak_speed_; }
  inline float decel_disp_threshold() const { return decel_disp_threshold_; }
  inline const Eigen::Vector2f& start_loc() const { return start_loc_; }

 private:
  Eigen::Vector2f start_loc_;
  float target_displacement_;
  float peak_speed_;
  float decel_disp_threshold_;
};

}  // namespace navigation::toc

#endif  // CS393R_NAVIGATION_TOC_H_
