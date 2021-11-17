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

// dynamic constraints
constexpr float kMaxSpeed = 1.0f;                                                    // m / s
constexpr float kMaxAccel = 4.0f;                                                    // m / s^2
constexpr float kMaxDecel = -4.0f;                                                   // m / s^2
const float kBrakingDistance = (kMaxSpeed * kMaxSpeed) / (2 * std::abs(kMaxDecel));  // m

// dimensions + kinematic constraints
constexpr float kCarXLength = 0.5;                             // m
constexpr float kBaseToFront = 0.42;                           // m
constexpr float kBaseToBack = kCarXLength - kBaseToFront;      // m
constexpr float kBaseToSide = 0.14;                            // m
constexpr float kSafetyMargin = 0.05;                          // m
constexpr float kWheelBase = 0.33;                             // m
const float kMinSteeringAngle = std::atan2(kWheelBase, -1.1);  // rad
const float kMaxSteeringAngle = std::atan2(kWheelBase, 1.1);   // rad

// TODO: extern bc of one-def-rule?

// Define the boundaries of the car in terms of its corners
// in each quadrant of the base_link reference frame.
const Eigen::Vector2f q1_corner(kBaseToFront + kSafetyMargin, kBaseToSide + kSafetyMargin);
const Eigen::Vector2f q2_corner(-kBaseToBack - kSafetyMargin, kBaseToSide + kSafetyMargin);
const Eigen::Vector2f q3_corner(-kBaseToBack - kSafetyMargin, -kBaseToSide - kSafetyMargin);
const Eigen::Vector2f q4_corner(kBaseToFront + kSafetyMargin, -kBaseToSide - kSafetyMargin);

}  // namespace constants
}  // namespace navigation

#endif  // NAVIGATION_CONSTANTS_HH_
