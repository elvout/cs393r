// Force `assert` statements to work.
// Some of the compiler flags in the shared libraries define NDEBUG.
// TODO: do this more gracefully
#ifdef NDEBUG
#undef NDEBUG
#undef assert
#endif
#include <cassert>

#include <algorithm>
#include <memory>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "navigation.h"
#include "shared/math/math_util.h"
#include "visualization/visualization.h"

namespace {

// Epsilon value for handling limited numerical precision
constexpr float kEpsilon = 1e-5;

// Car dimensions
constexpr float kCarXLength = 0.5;                         // m
constexpr float kBaseToFront = 0.42;                       // m
constexpr float kBaseToBack = kCarXLength - kBaseToFront;  // m
constexpr float kBaseToSide = 0.14;                        // m
constexpr float kSafetyMargin = 0.1;                       // m

// Define the boundaries of the car in terms of its corners
// in each quadrant of the base_link reference frame.
const Eigen::Vector2f q1_corner(kBaseToFront + kSafetyMargin, kBaseToSide + kSafetyMargin);
const Eigen::Vector2f q2_corner(-kBaseToBack - kSafetyMargin, kBaseToSide + kSafetyMargin);
const Eigen::Vector2f q3_corner(-kBaseToBack - kSafetyMargin, -kBaseToSide - kSafetyMargin);
const Eigen::Vector2f q4_corner(kBaseToFront + kSafetyMargin, -kBaseToSide - kSafetyMargin);

/**
 * Return an angle specfied in radians constrained to the range [0, 2PI).
 *
 * TODO: move this to the shared math library?
 */
template <typename T>
T constrainAngle(T angle) {
  static_assert(std::is_floating_point<T>::value, "");

  angle = fmod(angle, M_2PI);
  // `angle` can still be negative, but its absolute value will be < 2PI
  return fmod(angle + M_2PI, M_2PI);
}

/**
 * Returns the angular distance in radians between the base link of the
 * car and an arbitrary point as viewed from the center of turning.
 *
 * `point` is a coordinate in the base link reference frame.
 *
 * `radius` is the turning radius of the car. A positive value indicates
 * a left turn and a negative value indicates a right turn.
 */
float angularDistanceToPoint(Eigen::Vector2f point, const float radius) {
  Eigen::Vector2f center_to_base(0, -radius);

  if (radius == std::numeric_limits<float>::infinity()) {
    center_to_base.y() = -1e6f;
  }

  // Transform vectors into the reference frame of the center of turning,
  // in which the vector from the center of turning to the base link is
  // the x-axis.
  Eigen::Rotation2Df rot(std::asin(math_util::Sign(radius)));
  point = rot * (center_to_base + point);
  center_to_base = rot * center_to_base;

  assert(center_to_base.x() >= 0);
  // Slight precision issues for large radii, check the ratio of y / x
  // instead of the magnitude of y directly.
  assert(std::abs(center_to_base.y() / center_to_base.x()) < kEpsilon);

  float angular_dist = std::atan2(point.y(), point.x());
  angular_dist = constrainAngle(angular_dist);

  // `angular_dist` describes a counterclockwise angle from the x-axis in the
  // center-of-turning reference frame. In the case of a right turn, return
  // the clockwise angle.
  if (radius >= 0) {
    return angular_dist;
  } else {
    return M_2PI - angular_dist;
  }
}
}  // namespace

namespace navigation {

/**
 * Constructs a `PathOption` using the provided data.
 *
 * The constructed `PathOption` will store the maximum travelable
 * distance along a `curvature` arc before either colliding with an
 * obstacle or moving away from the `target`.
 */
PathOption::PathOption(const float curvature,
                       const std::vector<Eigen::Vector2f>& point_cloud,
                       const Eigen::Vector2f& target)
    : curvature(curvature),
      clearance(0),
      free_path_length(0),
      free_path_subtended_angle(0),
      closest_point_to_target(0, 0) {
  if (curvature == 0) {
    // Edge case: The car is driving straight.
    // An obstace can only hit the front of the car.

    const float min_y = q4_corner.y();
    const float max_y = q1_corner.y();
    assert(min_y < max_y);

    // Don't travel further than the target's `x` coordinate,
    // otherwise the car starts moving away from the target.
    free_path_length = target.x();

    for (const auto& point : point_cloud) {
      bool collision = min_y <= point.y() && point.y() <= max_y;
      if (collision) {
        // Assumes the point is in front of the car.
        float dist = point.x() - kBaseToFront;
        free_path_length = std::min(free_path_length, dist);
      }
    }
    closest_point_to_target = Eigen::Vector2f(free_path_length, 0);
  } else {
    // General case: The car is making a turn.

    const float turning_radius = 1 / curvature;
    const Eigen::Vector2f center_of_turning(0, turning_radius);

    // The angle of the closest point along the turning arc to the target.
    // Moving past this angle is (probably) counterproductive as the car
    // will begin to move away from the target.
    free_path_subtended_angle = angularDistanceToPoint(target, turning_radius);

    // Subroutine to calculate the subtended angle of the base link when the car
    // collides with a point and update the free path subtended angle accordingly.
    auto update_free_path_angle = [&](const Eigen::Vector2f& inner_corner,
                                      const Eigen::Vector2f& outer_corner,
                                      const Eigen::Vector2f& point) {
      const float point_radius = (center_of_turning - point).norm();
      const float angle_to_point = angularDistanceToPoint(point, turning_radius);

      const float inner_radius = (center_of_turning - inner_corner).norm();
      const float outer_radius = (center_of_turning - outer_corner).norm();
      assert(inner_radius < outer_radius);

      const bool collision = inner_radius <= point_radius && point_radius <= outer_radius;
      if (collision) {
        // Estimate where the point lies between the two radii as a proportion.
        const float p = (point_radius - inner_radius) / (outer_radius / inner_radius);
        // Calculate the corresponding point on the side of the car.
        const Eigen::Vector2f approx_hit_point_on_car = (1 - p) * inner_corner + p * outer_corner;
        const float angle_between_base_and_hit_point =
            angularDistanceToPoint(approx_hit_point_on_car, turning_radius);

        float path_subtended_angle =
            constrainAngle(angle_to_point - angle_between_base_and_hit_point);
        free_path_subtended_angle = std::min(free_path_subtended_angle, path_subtended_angle);
      }
    };

    for (const auto& point : point_cloud) {
      if (turning_radius > 0) {
        update_free_path_angle(q1_corner, q4_corner, point);  // Front side hit
        update_free_path_angle(q2_corner, q1_corner, point);  // Left side hit
      } else {
        update_free_path_angle(q4_corner, q1_corner, point);  // Front side hit
        update_free_path_angle(q3_corner, q4_corner, point);  // Right side hit
      }

      // TODO: invalid assertion? What if this path would head directly into an obstacle?
      // assert(free_path_subtended_angle >= 0);
    }

    free_path_length = free_path_subtended_angle * std::abs(turning_radius);
    closest_point_to_target =
        Eigen::Vector2f(std::abs(turning_radius) * std::sin(free_path_subtended_angle),
                        turning_radius - turning_radius * std::cos(free_path_subtended_angle));
  }
}

/**
 * Visualize this path with an arc of length `free_path_length`.
 *
 * `msg` should be the `VisualizationMsg` for the robot's local reference frame.
 *
 * `color` is a hex color code for the path.
 */
void PathOption::visualize(amrl_msgs::VisualizationMsg& msg, uint32_t color) const {
  if (curvature == 0) {
    visualization::DrawLine(Eigen::Vector2f(0, 0), closest_point_to_target, color, msg);
  } else {
    const float turning_radius = 1 / curvature;
    const Eigen::Vector2f center_of_turning(0, turning_radius);
    if (turning_radius > 0) {
      visualization::DrawArc(center_of_turning, turning_radius, -M_PI / 2,
                             -M_PI / 2 + free_path_subtended_angle, color, msg);
    } else {
      // DrawArc expects a counterclockwise arc
      visualization::DrawArc(center_of_turning, std::abs(turning_radius),
                             M_PI / 2 - free_path_subtended_angle, M_PI / 2, color, msg);
    }
  }
}

}  // namespace navigation
