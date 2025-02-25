/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testPathFactor.cpp
 * @date February 2025
 * @author Akshay Krishnan and Frank Dellaert
 * @brief Unit tests for PathFactor using Rot3 as the Lie group.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/EdgeKey.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sfm/PathFactor.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace std::placeholders;

//--------------------------------------------------------------------------
// SO(3) tests
//--------------------------------------------------------------------------

namespace SO3_example {
// Define initial rotations R1, R2, R3 about the Z-axis.
Rot3 R1 = Rot3::Rz(M_PI / 6);  // 30 degrees
Rot3 R2 = Rot3::Rz(M_PI / 4);  // 45 degrees
Rot3 R3 = Rot3::Rz(M_PI / 3);  // 60 degrees

// Calculate ground truth measurements
Rot3 R12 = R1.between(R2);
Rot3 R23 = R2.between(R3);
Rot3 R13 = R1.between(R3);
}  // namespace SO3_example

//--------------------------------------------------------------------------
// Test a simple path
//--------------------------------------------------------------------------
TEST(PathFactor, SimplePath) {
  using namespace SO3_example;

  // Create a simple path: from node 1->2 and 2->3.
  EdgeKey e12(1, 2), e23(2, 3);

  // Create a PathFactor with measured rotation equal to the predicted one.
  PathFactor<Rot3> factor(1, 3, R13, {e12, e23});

  // Populate a Values object with the appropriate measurements.
  Values values{{e12, genericValue(R12)}, {e23, genericValue(R23)}};

  // Check the factor error.
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);

  // Use the macro to check the correctness of the Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Test with a noise model
//--------------------------------------------------------------------------
TEST(PathFactor, NoiseModel) {
  using namespace SO3_example;
  EdgeKey e12(1, 2), e23(2, 3);

  // Same as above but add noise model.
  PathFactor<Rot3> factor(1, 3, R13, {e12, e23},
                          noiseModel::Isotropic::Sigma(3, 0.1));
  Values values{{e12, genericValue(R12)}, {e23, genericValue(R23)}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Reverse one of the edges
//--------------------------------------------------------------------------
TEST(PathFactor, WithReversal) {
  using namespace SO3_example;
  EdgeKey e12(1, 2), e32(3, 2);
  PathFactor<Rot3> factor(1, 3, R13, {e12, e32});
  Values values{{e12, genericValue(R12)}, {e32, genericValue(R23.inverse())}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Reverse one of the edges
//--------------------------------------------------------------------------
TEST(PathFactor, ReverseBothEdges) {
  using namespace SO3_example;
  EdgeKey e21(2, 1), e32(3, 2);
  PathFactor<Rot3> factor(1, 3, R13, {e21, e32});
  Values values{{e21, genericValue(R12.inverse())},
                {e32, genericValue(R23.inverse())}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// SE(3) tests below
//--------------------------------------------------------------------------
namespace SE3_example {
using namespace SO3_example;
// Define initial rotations T1, T2, T3 about the Z-axis.
Pose3 T1(R1, Point3(1, 2, 3));
Pose3 T2(R2, Point3(4, 5, 6));
Pose3 T3(R3, Point3(7, 8, 9));

// Calculate ground truth measurements
Pose3 T12 = T1.between(T2);
Pose3 T23 = T2.between(T3);
Pose3 T13 = T1.between(T3);
}  // namespace SE3_example

//--------------------------------------------------------------------------
// Test a simple path
//--------------------------------------------------------------------------
TEST(PathFactorSE3, SimplePath) {
  using namespace SE3_example;
  EdgeKey e12(1, 2), e23(2, 3);
  PathFactor<Pose3> factor(1, 3, T13, {e12, e23});
  Values values{{e12, genericValue(T12)}, {e23, genericValue(T23)}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Test with a noise model
//--------------------------------------------------------------------------
TEST(PathFactorSE3, NoiseModel) {
  using namespace SE3_example;
  EdgeKey e12(1, 2), e23(2, 3);
  PathFactor<Pose3> factor(1, 3, T13, {e12, e23},
                           noiseModel::Isotropic::Sigma(6, 0.1));
  Values values{{e12, genericValue(T12)}, {e23, genericValue(T23)}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Reverse one of the edges
//--------------------------------------------------------------------------
TEST(PathFactorSE3, WithReversal) {
  using namespace SE3_example;
  EdgeKey e12(1, 2), e32(3, 2);
  PathFactor<Pose3> factor(1, 3, T13, {e12, e32});
  Values values{{e12, genericValue(T12)}, {e32, genericValue(T23.inverse())}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Reverse one of the edges
//--------------------------------------------------------------------------
TEST(PathFactorSE3, ReverseBothEdges) {
  using namespace SE3_example;
  EdgeKey e21(2, 1), e32(3, 2);
  PathFactor<Pose3> factor(1, 3, T13, {e21, e32});
  Values values{{e21, genericValue(T12.inverse())},
                {e32, genericValue(T23.inverse())}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Pose2 tests
//--------------------------------------------------------------------------
namespace Pose2_example {
// Define initial poses
Pose2 P1(1, 2, 0);             // x=1, y=2, theta=0 radians
Pose2 P2(3, 4, M_PI / 4);      // x=3, y=4, theta=45 degrees
Pose2 P3(5, 6, M_PI / 2);      // x=5, y=6, theta=90 degrees
Pose2 P4(7, 8, 3 * M_PI / 4);  // x=7, y=8, theta=135 degrees
Pose2 P5(9, 10, M_PI);         // x=9, y=10, theta=180 degrees

// Calculate ground truth measurements
Pose2 P12 = P1.between(P2);
Pose2 P23 = P2.between(P3);
Pose2 P34 = P3.between(P4);
Pose2 P45 = P4.between(P5);
Pose2 P15 = P1.between(P5);
}  // namespace Pose2_example

//--------------------------------------------------------------------------
// Test a simple path with Pose2
//--------------------------------------------------------------------------
TEST(PathFactorPose2, SimplePath) {
  using namespace Pose2_example;
  EdgeKey e12(1, 2), e23(2, 3), e34(3, 4), e45(4, 5);
  PathFactor<Pose2> factor(1, 5, P15, {e12, e23, e34, e45});
  Values values{{e12, genericValue(P12)},
                {e23, genericValue(P23)},
                {e34, genericValue(P34)},
                {e45, genericValue(P45)}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Test reversing two edges
//--------------------------------------------------------------------------
TEST(PathFactorPose2, ReverseTwo) {
  using namespace Pose2_example;
  EdgeKey e12(1, 2), e32(3, 2), e34(3, 4), e54(5, 4);
  PathFactor<Pose2> factor(1, 5, P15, {e12, e32, e34, e54});
  Values values{{e12, genericValue(P12)},
                {e32, genericValue(P23.inverse())},
                {e34, genericValue(P34)},
                {e54, genericValue(P45.inverse())}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Test reversing measurement
//--------------------------------------------------------------------------
TEST(PathFactorPose2, ReverseMeasurement) {
  using namespace Pose2_example;
  EdgeKey e12(1, 2), e23(2, 3), e34(3, 4), e45(4, 5);
  PathFactor<Pose2> factor(5, 1, P15.inverse(), {e45, e34, e23, e12});
  Values values{{e12, genericValue(P12)},
                {e23, genericValue(P23)},
                {e34, genericValue(P34)},
                {e45, genericValue(P45)}};
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
