#ifndef SRC_MODELS_CONSTRAINTS_HH_
#define SRC_MODELS_CONSTRAINTS_HH_

#include <cmath>
#include "eigen3/Eigen/Dense"

namespace models {

// Epsilon value for handling limited numerical precision.
constexpr float kEpsilon = 1e-5;

// Dimensions and Kinematic Constraints
constexpr float kCarXLength = 0.5;                             // m
constexpr float kBaseToFront = 0.42;                           // m
constexpr float kBaseToBack = kCarXLength - kBaseToFront;      // m
constexpr float kBaseToSide = 0.14;                            // m
constexpr float kSafetyMargin = 0.07;                          // m
constexpr float kWheelBase = 0.33;                             // m
const float kMinSteeringAngle = std::atan(kWheelBase / -1.1);  // rad
const float kMaxSteeringAngle = std::atan(kWheelBase / 1.1);   // rad
const Eigen::Vector2f kLaserOffset(0.2, 0);
// Define the boundaries of the car in terms of its corners
// in each quadrant of the base_link reference frame.
const Eigen::Vector2f q1_corner(kBaseToFront + kSafetyMargin, kBaseToSide + kSafetyMargin);
const Eigen::Vector2f q2_corner(-kBaseToBack - kSafetyMargin, kBaseToSide + kSafetyMargin);
const Eigen::Vector2f q3_corner(-kBaseToBack - kSafetyMargin, -kBaseToSide - kSafetyMargin);
const Eigen::Vector2f q4_corner(kBaseToFront + kSafetyMargin, -kBaseToSide - kSafetyMargin);

// Dynamic Constraints
constexpr float kMaxSpeed = 1.0f;                                                    // m / s
constexpr float kMaxAccel = 4.0f;                                                    // m / s^2
constexpr float kMaxDecel = -4.0f;                                                   // m / s^2
const float kBrakingDistance = (kMaxSpeed * kMaxSpeed) / (2 * std::abs(kMaxDecel));  // m

}  // namespace models

#endif  // SRC_MODELS_CONSTRAINTS_HH_
