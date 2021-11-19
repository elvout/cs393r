#ifndef SRC_MODELS_MOTION_HH_
#define SRC_MODELS_MOTION_HH_

#include <utility>
#include "eigen3/Eigen/Dense"

namespace models {

/**
 * Return a sample from the Motion Model.
 */
std::pair<Eigen::Vector2f, float> SampleMotionModel(const Eigen::Vector2f& expected_disp,
                                                    const float expected_angle);

/**
 * Evaluate the log-likelihood of the motion model at the sample pose.
 */
double EvalLogMotionModel(const Eigen::Vector2f& expected_disp,
                          const double expected_angle,
                          const Eigen::Vector2f& sample_disp,
                          const double sample_angle);
}  // namespace models

#endif  // SRC_MODELS_MOTION_HH_
