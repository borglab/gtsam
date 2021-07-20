/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testTranslationFactor.cpp
 *  @brief Unit tests for TranslationFactor Class
 *  @author Frank dellaert
 *  @date March 2020
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/sfm/TranslationFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// Create a noise model for the chordal error
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(3, 0.05));

// Keys are deliberately *not* in sorted order to test that case.
static const Key kKey1(2), kKey2(1);
static const Unit3 kMeasured(1, 0, 0);

/* ************************************************************************* */
TEST(TranslationFactor, Constructor) {
  TranslationFactor factor(kKey1, kKey2, kMeasured, model);
}

/* ************************************************************************* */
TEST(TranslationFactor, ZeroError) {
  // Create a factor
  TranslationFactor factor(kKey1, kKey2, kMeasured, model);

  // Set the linearization
  Point3 T1(1, 0, 0), T2(2, 0, 0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(T1, T2));

  // Verify we get the expected error
  Vector expected = (Vector3() << 0, 0, 0).finished();
  EXPECT(assert_equal(expected, actualError, 1e-9));
}

/* ************************************************************************* */
TEST(TranslationFactor, NonZeroError) {
  // create a factor
  TranslationFactor factor(kKey1, kKey2, kMeasured, model);

  // set the linearization
  Point3 T1(0, 1, 1), T2(0, 2, 2);

  // use the factor to calculate the error
  Vector actualError(factor.evaluateError(T1, T2));

  // verify we get the expected error
  Vector expected = (Vector3() << -1, 1 / sqrt(2), 1 / sqrt(2)).finished();
  EXPECT(assert_equal(expected, actualError, 1e-9));
}

/* ************************************************************************* */
Vector factorError(const Point3 &T1, const Point3 &T2,
                   const TranslationFactor &factor) {
  return factor.evaluateError(T1, T2);
}

TEST(TranslationFactor, Jacobian) {
  // Create a factor
  TranslationFactor factor(kKey1, kKey2, kMeasured, model);

  // Set the linearization
  Point3 T1(1, 0, 0), T2(2, 0, 0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(T1, T2, H1Actual, H2Actual);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<Vector, Point3>(
      std::bind(&factorError, std::placeholders::_1, T2, factor), T1);
  H2Expected = numericalDerivative11<Vector, Point3>(
      std::bind(&factorError, T1, std::placeholders::_1, factor), T2);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
