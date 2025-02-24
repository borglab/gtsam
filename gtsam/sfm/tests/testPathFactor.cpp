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
#include <gtsam/geometry/Rot3.h>
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

namespace rotation_example {
// Define initial rotations R1, R2, R3 about the Z-axis.
Rot3 R1 = Rot3::Rz(M_PI / 6);  // 30 degrees
Rot3 R2 = Rot3::Rz(M_PI / 4);  // 45 degrees
Rot3 R3 = Rot3::Rz(M_PI / 3);  // 60 degrees

// Calculate ground truth measurements
Rot3 R12 = R1.between(R2);
Rot3 R23 = R2.between(R3);
Rot3 R13 = R1.between(R3);
}  // namespace rotation_example

//--------------------------------------------------------------------------
// Test a simple path
//--------------------------------------------------------------------------
TEST(PathFactor, SimplePath) {
  using namespace rotation_example;

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
  using namespace rotation_example;

  // Create a simple path: from node 1->2 and 2->3.
  EdgeKey e12(1, 2), e23(2, 3);

  // Create a PathFactor with measured rotation equal to the predicted one.
  PathFactor<Rot3> factor(1, 3, R13, {e12, e23},
                          noiseModel::Isotropic::Sigma(3, 0.1));

  // Populate a Values object with the appropriate measurements.
  Values values{{e12, genericValue(R12)}, {e23, genericValue(R23)}};

  // Check the factor error.
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);

  // Use the macro to check the correctness of the Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Reverse one of the edges
//--------------------------------------------------------------------------
TEST(PathFactor, WithReversal) {
  using namespace rotation_example;
  EdgeKey e12(1, 2), e23(2, 3);
  PathFactor<Rot3> factor(1, 3, R13, {e12, e23});
  Values values{{e12, genericValue(R12)},
                {e23.reversed(), genericValue(R23.inverse())}};

  // Check the factor error.
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);

  // Use the macro to check the correctness of the Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//--------------------------------------------------------------------------
// Reverse one of the edges
//--------------------------------------------------------------------------
TEST(PathFactor, ReverseBothedges) {
  using namespace rotation_example;
  EdgeKey e12(1, 2), e23(2, 3);
  PathFactor<Rot3> factor(1, 3, R13, {e12, e23});
  Values values{{e12.reversed(), genericValue(R12.inverse())},
                {e23.reversed(), genericValue(R23.inverse())}};

  // Check the factor error.
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-6);

  // Use the macro to check the correctness of the Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
