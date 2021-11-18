#include "motion.hh"

#include <utility>
#include "config_reader/config_reader.h"
#include "eigen3/Eigen/Dense"
#include "models/normdist.hh"
#include "shared/math/math_util.h"
#include "shared/util/random.h"

namespace {
// Read hyperparameters from configuration file.
CONFIG_DOUBLE(k1, "motion_model_k1");
CONFIG_DOUBLE(k2, "motion_model_k2");
CONFIG_DOUBLE(k3, "motion_model_k3");
CONFIG_DOUBLE(k4, "motion_model_k4");

util_random::Random rng_;
}  // namespace

namespace models {

std::pair<Eigen::Vector2f, float> SampleMotionModel(const Eigen::Vector2f& expected_disp,
                                                    const float expected_angle) {
  const float disp_norm = expected_disp.norm();
  const float angle_norm = std::abs(expected_angle);

  const double translate_std = CONFIG_k1 * disp_norm + CONFIG_k2 * angle_norm;
  const double rotate_std = CONFIG_k3 * disp_norm + CONFIG_k4 * angle_norm;

  const Eigen::Vector2f translate_err(rng_.Gaussian(0, translate_std),
                                      rng_.Gaussian(0, translate_std));
  const double rotate_err = rng_.Gaussian(0, rotate_std);

  return std::make_pair<Eigen::Vector2f, float>(expected_disp + translate_err,
                                                expected_angle + rotate_err);
}

double EvalLogMotionModel(const Eigen::Vector2f& expected_disp,
                          const double expected_angle,
                          const Eigen::Vector2f& sample_disp,
                          const double sample_angle) {
  const double expected_disp_n = expected_disp.norm();
  const double expected_angle_n = std::abs(math_util::ReflexToConvexAngle(expected_angle));
  const double disp_std = CONFIG_k1 * expected_disp_n + CONFIG_k2 * expected_angle_n;
  const double rot_std = CONFIG_k3 * expected_disp_n + CONFIG_k4 * expected_angle_n;

  const double x_noise = sample_disp.x() - expected_disp.x();
  const double y_noise = sample_disp.y() - expected_disp.y();
  const double theta_noise = math_util::ReflexToConvexAngle(sample_angle - expected_angle);

  const double log_px = LnOfNormalPdf(x_noise, 0, disp_std);
  const double log_py = LnOfNormalPdf(y_noise, 0, disp_std);
  const double log_ptheta = LnOfNormalPdf(theta_noise, 0, rot_std);

  return log_px + log_py + log_ptheta;
}

}  // namespace models
