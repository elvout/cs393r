#include <stdio.h>
#include <cmath>
#include <iostream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using std::cos;
using std::cout;
using std::endl;
using std::sin;
using std::sqrt;

using Eigen::Matrix2f;
using Eigen::Rotation2Df;
using Eigen::Vector2f;

void DemoBasics() {
  cout << "Basic initialization" << endl;
  cout << "Initialize a 2D vector v1." << endl;
  Vector2f v1(1.0, 2.0);

  cout << "Read elements of the 3D vector:" << endl
       << "v1.x = " << v1.x() << endl
       << "v1.y = " << v1.y() << endl;

  cout << "Write 10 to the x coordinate of v1:" << endl;
  v1.x() = 10.0;
  cout << "v1.x = " << v1.x() << endl;

  cout << "Print the vector to stdout:\n" << v1 << endl;

  cout << "Initialize a 2x2 matrix m1." << endl;
  Matrix2f m1;
  // clang-format off
  m1 << 0, 2,
        3, 0;
  // clang-format on
  cout << "m1 = " << endl << m1 << endl;

  cout << "Multiply matrix times vector." << endl;
  Vector2f v2 = m1 * v1;
  cout << "Resulting vector:\n" << v2 << endl;
}

void DemoRotations() {
  cout << "Rotations demonstration" << endl;
  // Note that pi/4 radians is 45 degrees.
  float angle1 = M_PI / 4.0;
  cout << "angle1 = " << angle1 << " radians = " << angle1 / M_PI * 180.0 << " degrees." << endl;

  cout << "Create a rotation" << endl;
  Rotation2Df r1(angle1);

  cout << "Apply that rotation to a vector" << endl;
  Vector2f v1(1.0, 0);
  Vector2f v2 = r1 * v1;
  cout << v2 << "\n";

  cout << "Convert rotation to a matrix" << endl;
  Matrix2f m1 = r1.toRotationMatrix();
  cout << "m1: \n" << m1 << "\n";
}

void TestAggregate() {
  Vector2f init_obs(15, 0);

  Vector2f odom_disp1(4, -2);
  double odom_angle1 = -45 / 180.0 * M_PI;

  Vector2f odom_disp2(0, sqrt(50));
  double odom_angle2 = (45.0 + 30) / 180 * M_PI;

  Vector2f aggregate_disp(0, 0);
  double aggregate_angle = 0;

  aggregate_disp += odom_disp1;
  aggregate_angle += odom_angle1;

  aggregate_disp += Rotation2Df(aggregate_angle) * odom_disp2;
  aggregate_angle += odom_angle2;

  std::cout << "pos: [" << aggregate_disp.x() << ", " << aggregate_disp.y() << " | "
            << (aggregate_angle * 180 / M_PI) << "]\n";

  Vector2f expected_point(6, -3);
  expected_point = Rotation2Df(-30.0 / 180 * M_PI) * expected_point;

  printf("local ref: [%.4f, %.4f]\n", expected_point.x(), expected_point.y());

  expected_point = Rotation2Df(aggregate_angle) * expected_point;
  expected_point += aggregate_disp;

  printf("local ref: [%.4f, %.4f]\n", expected_point.x(), expected_point.y());
}

int main() {
  // cout << "Basics: Vectors and Matrices.\n";
  // DemoBasics();

  // cout << "\n\n\nDifferent representations of rotation.\n";
  // DemoRotations();

  TestAggregate();

  return 0;
}
