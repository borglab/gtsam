/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testPointCorrespondenceFactor.cpp
 * @brief Tests for PointCorrespondenceFactor.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PointCorrespondenceFactor.h>

using namespace gtsam;

/* ************************************************************************* */
namespace rot2_fixture {

const Key kKey = 1;
const Rot2 kTransform = Rot2::fromAngle(0.4);
const Point2 kSourcePoint(1.2, -0.7);
const Point2 kMeasuredPoint(0.3, 0.8);

// Verifies the Rot2 residual, exact correspondence, and analytic Jacobian.
TEST(PointCorrespondenceFactor, Rot2) {
  const auto model = noiseModel::Unit::Create(2);
  const PointCorrespondenceFactor<Rot2> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected = kTransform.rotate(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(2, H.rows());
  EXPECT_LONGS_EQUAL(1, H.cols());

  const PointCorrespondenceFactor<Rot2> exactFactor(
      kKey, kSourcePoint, kTransform.rotate(kSourcePoint), model);
  EXPECT(assert_equal(Vector2::Zero(), exactFactor.evaluateError(kTransform)));

  Values values;
  values.insert(kKey, kTransform);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

}  // namespace rot2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace rot3_fixture {

const Key kKey = 2;
const Rot3 kTransform = Rot3::Expmap(Vector3(0.2, -0.1, 0.3));
const Point3 kSourcePoint(1.2, -0.7, 0.5);
const Point3 kMeasuredPoint(0.3, 0.8, -0.4);

// Verifies the Rot3 residual, exact correspondence, and analytic Jacobian.
TEST(PointCorrespondenceFactor, Rot3) {
  const auto model = noiseModel::Unit::Create(3);
  const PointCorrespondenceFactor<Rot3> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected = kTransform.rotate(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const PointCorrespondenceFactor<Rot3> exactFactor(
      kKey, kSourcePoint, kTransform.rotate(kSourcePoint), model);
  EXPECT(assert_equal(Vector3::Zero(), exactFactor.evaluateError(kTransform)));

  Values values;
  values.insert(kKey, kTransform);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

}  // namespace rot3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace pose2_fixture {

const Key kKey = 3;
const Pose2 kTransform(0.4, -0.2, 0.3);
const Point2 kSourcePoint(1.2, -0.7);
const Point2 kMeasuredPoint(0.3, 0.8);

// Verifies the Pose2 residual, exact correspondence, and analytic Jacobian.
TEST(PointCorrespondenceFactor, Pose2) {
  const auto model = noiseModel::Unit::Create(2);
  const PointCorrespondenceFactor<Pose2> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected =
      kTransform.transformFrom(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(2, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const PointCorrespondenceFactor<Pose2> exactFactor(
      kKey, kSourcePoint, kTransform.transformFrom(kSourcePoint), model);
  EXPECT(assert_equal(Vector2::Zero(), exactFactor.evaluateError(kTransform)));

  Values values;
  values.insert(kKey, kTransform);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

}  // namespace pose2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace pose3_fixture {

const Key kKey = 4;
const Pose3 kTransform(Rot3::Expmap(Vector3(0.2, -0.1, 0.3)),
                       Point3(0.4, -0.2, 0.6));
const Point3 kSourcePoint(1.2, -0.7, 0.5);
const Point3 kMeasuredPoint(0.3, 0.8, -0.4);

// Verifies the Pose3 residual, exact correspondence, and analytic Jacobian.
TEST(PointCorrespondenceFactor, Pose3) {
  const auto model = noiseModel::Unit::Create(3);
  const PointCorrespondenceFactor<Pose3> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected =
      kTransform.transformFrom(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(6, H.cols());

  const PointCorrespondenceFactor<Pose3> exactFactor(
      kKey, kSourcePoint, kTransform.transformFrom(kSourcePoint), model);
  EXPECT(assert_equal(Vector3::Zero(), exactFactor.evaluateError(kTransform)));

  Values values;
  values.insert(kKey, kTransform);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies that a full information matrix determines the factor cost.
TEST(PointCorrespondenceFactor, FullInformationMatrix) {
  Matrix3 information;
  information << 4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0;
  const auto model = noiseModel::Gaussian::Information(information);
  const PointCorrespondenceFactor<Pose3> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Values values;
  values.insert(kKey, kTransform);
  const Vector error = factor.evaluateError(kTransform);
  const double expected = 0.5 * error.dot(information * error);
  EXPECT_DOUBLES_EQUAL(expected, factor.error(values), 1e-9);
}

}  // namespace pose3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
