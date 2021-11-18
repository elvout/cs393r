#include "normdist.hh"

#include <cmath>

namespace models {

double NormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2pi))
  constexpr double inv_sqrt2pi = 0.39894228040143267794;

  const double z = (val - mean) / stddev;
  return inv_sqrt2pi / stddev * std::exp(-0.5 * z * z);
}

double NormalCdf(const double val, const double mean, const double stddev) {
  // precomputed value for (1 / sqrt(2))
  constexpr double inv_sqrt2 = 0.70710678118654752440;

  const double z = (val - mean) / stddev;
  return 0.5 + 0.5 * std::erf(z * inv_sqrt2);
}

double LnOfNormalPdf(const double val, const double mean, const double stddev) {
  // precomputed value for log(1 / sqrt(2pi))
  constexpr double log_inv_sqrt2pi = -0.91893853320467274178;

  const double z = (val - mean) / stddev;
  return log_inv_sqrt2pi - std::log(stddev) - (z * z * 0.5);
}

}  // namespace models
