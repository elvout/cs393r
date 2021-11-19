#ifndef NAVIGATION_CONSTANTS_HH_
#define NAVIGATION_CONSTANTS_HH_

#include <cmath>
#include "eigen3/Eigen/Dense"

namespace navigation {
namespace constants {

// Epsilon value for handling limited numerical precision.
constexpr float kEpsilon = 1e-5;

constexpr float kUpdateFrequency = 20.0f;  // 1 / s
constexpr float kActuationLatency = 0.2f;  // s
constexpr float kControlHistorySize = kActuationLatency * kUpdateFrequency;

}  // namespace constants
}  // namespace navigation

#endif  // NAVIGATION_CONSTANTS_HH_
