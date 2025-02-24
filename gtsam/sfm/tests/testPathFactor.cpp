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
#include <gtsam/base/numericalDerivative.h>
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

//--------------------------------------------------------------------------
// Test: Factor Error
//--------------------------------------------------------------------------

TEST(PathFactor, Error) {
  // Create a simple path: from node 1->2 and 2->3.
  EdgeKey e12(1, 2), e23(2, 3);
  std::vector<EdgeKey> pathKeys = {e12, e23};

  // Define rotations.
  // R12: rotation about Z-axis by 30 degrees.
  Rot3 R12 = Rot3::Rz(30 * M_PI / 180.0);
  // R23: rotation about Z-axis by 45 degrees.
  Rot3 R23 = Rot3::Rz(45 * M_PI / 180.0);

  // The predicted overall rotation is R12 * R23.
  Rot3 predicted = R12.compose(R23);

  // Create a PathFactor with measured rotation equal to the predicted one.
  PathFactor<Rot3> factor(pathKeys, predicted);

  // Populate a Values object with the appropriate measurements.
  Values values{{e12, genericValue(R12)}, {e23, genericValue(R23)}};

  // Compute the factor error.
  double error_val = factor.error(values);
  EXPECT(abs(error_val) < 1e-6);
}

Vector3 residualFunc(const Rot3& R12, const Rot3& R32, const Rot3& R13) {
  Rot3 prediction = R12.compose(R32.inverse());
  Rot3 residual = R13.inverse().compose(prediction);
  return Rot3::Logmap(residual);
};

//--------------------------------------------------------------------------
// Test: Jacobian (Analytical vs. Numerical)
//--------------------------------------------------------------------------
TEST(PathFactor, Jacobian) {
  // Create a simple path: from node 1->2 and 2->3.
  EdgeKey e12(1, 2), e23(2, 3);
  std::vector<EdgeKey> pathKeys = {e12, e23};

  // Define rotations.
  Rot3 R12 = Rot3::Rz(30 * M_PI / 180.0);  // for edge e12 (forward)
  Rot3 R23 = Rot3::Rz(45 * M_PI /
                      180.0);  // for edge e23 (measurement stored reversed)

  // The predicted overall rotation is R12 * R23.
  Rot3 R13 = R12.compose(R23);

  // Create the PathFactor.
  PathFactor<Rot3> factor(pathKeys, R13);

  // Populate a Values object.
  Values values{{e12, genericValue(R12)}, {e23, genericValue(R23)}};

  // Use the macro to check the correctness of the Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
