/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testAttitudeFactor.cpp
 * @brief   Unit test for Rot3AttitudeFactor
 * @author  Frank Dellaert
 * @date   January 22, 2014
 */

#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind/bind.hpp>
#include <functional>
#include "gtsam/base/Matrix.h"
#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// *************************************************************************
TEST( Rot3AttitudeFactor, Constructor ) {

  // Example: pitch and roll of aircraft in an ENU Cartesian frame.
  // If pitch and roll are zero for an aerospace frame,
  // that means Z is pointing down, i.e., direction of Z = (0,0,-1)
  Unit3 bZ(0, 0, 1); // reference direction is body Z axis
  Unit3 nDown(0, 0, -1); // down, in ENU navigation frame, is "measurement"

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Rot3AttitudeFactor factor0(key, nDown, model);
  Rot3AttitudeFactor factor(key, nDown, model, bZ);
  EXPECT(assert_equal(factor0,factor,1e-5));

  // Create a linearization point at the zero-error point
  Rot3 nRb;
  EXPECT(assert_equal((Vector) Z_2x1,factor.evaluateError(nRb),1e-5));

  std::function<Vector(const Rot3&)> err_fn = [&factor](const Rot3& r){
	return factor.evaluateError(r, OptionalNone);
  };
  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Vector, Rot3>(err_fn, nRb);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(nRb, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

/* ************************************************************************* */
TEST(Rot3AttitudeFactor, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Rot3AttitudeFactor factor(0, nDown, model);

  // Copy assignable.
  EXPECT(std::is_copy_assignable<Rot3AttitudeFactor>::value);
  Rot3AttitudeFactor factor_copied = factor;
  EXPECT(assert_equal(factor, factor_copied));

  // Move assignable.
  EXPECT(std::is_move_assignable<Rot3AttitudeFactor>::value);
  Rot3AttitudeFactor factor_moved = std::move(factor_copied);
  EXPECT(assert_equal(factor, factor_moved));
}

// *************************************************************************
TEST( Pose3AttitudeFactor, Constructor ) {

  // Example: pitch and roll of aircraft in an ENU Cartesian frame.
  // If pitch and roll are zero for an aerospace frame,
  // that means Z is pointing down, i.e., direction of Z = (0,0,-1)
  Unit3 bZ(0, 0, 1); // reference direction is body Z axis
  Unit3 nDown(0, 0, -1); // down, in ENU navigation frame, is "measurement"

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Pose3AttitudeFactor factor0(key, nDown, model);
  Pose3AttitudeFactor factor(key, nDown, model, bZ);
  EXPECT(assert_equal(factor0,factor,1e-5));

  // Create a linearization point at the zero-error point
  Pose3 T(Rot3(), Point3(-5.0, 8.0, -11.0));
  EXPECT(assert_equal((Vector) Z_2x1,factor.evaluateError(T),1e-5));

  Matrix actualH1;

  std::function<Vector(const Pose3&)> err_fn = [&factor](const Pose3& p){
	return factor.evaluateError(p, OptionalNone);
  };
  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Vector,Pose3>(err_fn, T);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(T, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3AttitudeFactor, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Pose3AttitudeFactor factor(0, nDown, model);

  // Copy assignable.
  EXPECT(std::is_copy_assignable<Pose3AttitudeFactor>::value);
  Pose3AttitudeFactor factor_copied = factor;
  EXPECT(assert_equal(factor, factor_copied));

  // Move assignable.
  EXPECT(std::is_move_assignable<Pose3AttitudeFactor>::value);
  Pose3AttitudeFactor factor_moved = std::move(factor_copied);
  EXPECT(assert_equal(factor, factor_moved));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
