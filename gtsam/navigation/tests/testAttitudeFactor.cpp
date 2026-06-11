/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testAttitudeFactor.cpp
 * @brief   Unit test for AttitudeFactor variants
 * @author  Frank Dellaert
 * @date    January 22, 2014
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include <type_traits>

using namespace gtsam;

namespace {

template <class Factor>
bool defaultMeasuredAxisMatches(const Unit3& nDown,
                                const SharedNoiseModel& model) {
  Unit3 bMeasured(0, 0, 1);
  Factor factor0(1, nDown, model);
  Factor factor(1, nDown, model, bMeasured);
  return assert_equal(factor0, factor, 1e-5);
}

template <class Factor, class Value>
bool zeroResidualAndJacobianMatch(const Factor& factor, const Value& value) {
  if (!assert_equal((Vector)Z_2x1, factor.evaluateError(value), 1e-5)) {
    return false;
  }

  auto err_fn = [&factor](const Value& candidate) {
    return factor.evaluateError(candidate, OptionalNone);
  };
  Matrix expectedH = numericalDerivative11<Vector, Value>(err_fn, value);

  Matrix actualH;
  factor.evaluateError(value, actualH);
  return assert_equal(expectedH, actualH, 1e-8);
}

template <class Factor>
bool copyAndMoveWorks(const Factor& factor) {
  if (!std::is_copy_assignable<Factor>::value) {
    return false;
  }
  Factor factor_copied = factor;
  if (!assert_equal(factor, factor_copied)) {
    return false;
  }

  if (!std::is_move_assignable<Factor>::value) {
    return false;
  }
  Factor factor_moved = std::move(factor_copied);
  return assert_equal(factor, factor_moved);
}

template <int N, class Factor, class Value>
bool zeroResidualAndJacobianMatchDynamic(const Factor& factor,
                                         const Value& value) {
  if (!assert_equal((Vector)Z_2x1, factor.evaluateError(value), 1e-5)) {
    return false;
  }

  auto err_fn = [&factor](const Value& candidate) {
    return factor.evaluateError(candidate, OptionalNone);
  };
  Matrix expectedH = numericalDerivative11<Vector, Value, N>(err_fn, value);

  Matrix actualH;
  factor.evaluateError(value, actualH);
  return assert_equal(expectedH, actualH, 1e-8);
}

Se23 makeSe23(const Matrix32& x) { return Se23(Rot3(), x); }

ExtendedPose3d makeExtendedPose3d(const Matrix& x) {
  return ExtendedPose3d(Rot3(), x);
}

}  // namespace

/* ************************************************************************* */
TEST(AttitudeFactorRot3, ConstructorAndJacobian) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(defaultMeasuredAxisMatches<AttitudeFactor<Rot3>>(nDown, model));

  AttitudeFactor<Rot3> factor(1, nDown, model);
  EXPECT(zeroResidualAndJacobianMatch(factor, Rot3()));
}

/* ************************************************************************* */
TEST(AttitudeFactorRot3, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(copyAndMoveWorks(AttitudeFactor<Rot3>(0, nDown, model)));
}

/* ************************************************************************* */
TEST(AttitudeFactorPose3, ConstructorAndJacobian) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(defaultMeasuredAxisMatches<AttitudeFactor<Pose3>>(nDown, model));

  AttitudeFactor<Pose3> factor(1, nDown, model);
  EXPECT(zeroResidualAndJacobianMatch(factor,
                                      Pose3(Rot3(), Point3(-5.0, 8.0, -11.0))));
}

/* ************************************************************************* */
TEST(AttitudeFactorPose3, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(copyAndMoveWorks(AttitudeFactor<Pose3>(0, nDown, model)));
}

/* ************************************************************************* */
TEST(AttitudeFactorNavState, ConstructorAndJacobian) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(defaultMeasuredAxisMatches<AttitudeFactor<NavState>>(nDown, model));

  AttitudeFactor<NavState> factor(1, nDown, model);
  EXPECT(zeroResidualAndJacobianMatch(
      factor,
      NavState(Rot3(), Point3(-5.0, 8.0, -11.0), Vector3(0.2, -0.4, 0.6))));
}

/* ************************************************************************* */
TEST(AttitudeFactorNavState, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(copyAndMoveWorks(AttitudeFactor<NavState>(0, nDown, model)));
}

/* ************************************************************************* */
TEST(AttitudeFactorGal3, ConstructorAndJacobian) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(defaultMeasuredAxisMatches<AttitudeFactor<Gal3>>(nDown, model));

  AttitudeFactor<Gal3> factor(1, nDown, model);
  EXPECT(zeroResidualAndJacobianMatch(
      factor,
      Gal3(Rot3(), Point3(-5.0, 8.0, -11.0), Vector3(0.2, -0.4, 0.6), 1.25)));
}

/* ************************************************************************* */
TEST(AttitudeFactorGal3, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(copyAndMoveWorks(AttitudeFactor<Gal3>(0, nDown, model)));
}

/* ************************************************************************* */
TEST(AttitudeFactorSe23, ConstructorAndJacobian) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(defaultMeasuredAxisMatches<AttitudeFactor<Se23>>(nDown, model));

  Matrix32 x;
  x << -5.0, 0.2, 8.0, -0.4, -11.0, 0.6;
  AttitudeFactor<Se23> factor(1, nDown, model);
  EXPECT(zeroResidualAndJacobianMatch(factor, makeSe23(x)));
}

/* ************************************************************************* */
TEST(AttitudeFactorSe23, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(copyAndMoveWorks(AttitudeFactor<Se23>(0, nDown, model)));
}

/* ************************************************************************* */
TEST(AttitudeFactorExtendedPose3d, ConstructorAndJacobian) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(
      defaultMeasuredAxisMatches<AttitudeFactor<ExtendedPose3d>>(nDown, model));

  Matrix x(3, 3);
  x << -5.0, 0.2, 1.3, 8.0, -0.4, 2.1, -11.0, 0.6, -0.7;
  AttitudeFactor<ExtendedPose3d> factor(1, nDown, model);
  EXPECT(
      zeroResidualAndJacobianMatchDynamic<12>(factor, makeExtendedPose3d(x)));
}

/* ************************************************************************* */
TEST(AttitudeFactorExtendedPose3d, CopyAndMove) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  EXPECT(copyAndMoveWorks(AttitudeFactor<ExtendedPose3d>(0, nDown, model)));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
