/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testKnownLandmarkFactor.cpp
 * @brief Tests for KnownLandmarkFactor.
 */

#include <CppUnitLite/TestHarness.h>
#include <examples/MatrixWeightedLocalizationUtils.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/KnownLandmarkFactor.h>

using namespace gtsam;

/* ************************************************************************* */
namespace rot2_fixture {

const Key kKey = 1;
const Rot2 kRw = Rot2::fromAngle(0.4);
const Point2 wP(1.2, -0.7);
const Point2 measured_kP(0.3, 0.8);

// Verifies the Rot2 residual, exact correspondence, and analytic Jacobian.
TEST(KnownLandmarkFactor, Rot2) {
  const auto model = noiseModel::Unit::Create(2);
  const KnownLandmarkFactor<Rot2> factor(kKey, wP, measured_kP, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kRw, H);
  const Vector expected = kRw.rotate(wP) - measured_kP;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(2, H.rows());
  EXPECT_LONGS_EQUAL(1, H.cols());

  const KnownLandmarkFactor<Rot2> exactFactor(kKey, wP, kRw.rotate(wP), model);
  EXPECT(assert_equal(Vector2::Zero(), exactFactor.evaluateError(kRw)));

  Values values;
  values.insert(kKey, kRw);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

}  // namespace rot2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace rot3_fixture {

const Key kKey = 2;
const Rot3 kRw = Rot3::Expmap(Vector3(0.2, -0.1, 0.3));
const Point3 wP(1.2, -0.7, 0.5);
const Point3 measured_kP(0.3, 0.8, -0.4);

// Verifies the Rot3 residual, exact correspondence, and analytic Jacobian.
TEST(KnownLandmarkFactor, Rot3) {
  const auto model = noiseModel::Unit::Create(3);
  const KnownLandmarkFactor<Rot3> factor(kKey, wP, measured_kP, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kRw, H);
  const Vector expected = kRw.rotate(wP) - measured_kP;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const KnownLandmarkFactor<Rot3> exactFactor(kKey, wP, kRw.rotate(wP), model);
  EXPECT(assert_equal(Vector3::Zero(), exactFactor.evaluateError(kRw)));

  Values values;
  values.insert(kKey, kRw);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

}  // namespace rot3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace pose2_fixture {

const Key kKey = 3;
const Pose2 kTw(0.4, -0.2, 0.3);
const Point2 wP(1.2, -0.7);
const Point2 measured_kP(0.3, 0.8);

// Verifies the Pose2 residual, exact correspondence, and analytic Jacobian.
TEST(KnownLandmarkFactor, Pose2) {
  const auto model = noiseModel::Unit::Create(2);
  const KnownLandmarkFactor<Pose2> factor(kKey, wP, measured_kP, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTw, H);
  const Vector expected = kTw.transformFrom(wP) - measured_kP;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(2, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const KnownLandmarkFactor<Pose2> exactFactor(kKey, wP, kTw.transformFrom(wP),
                                               model);
  EXPECT(assert_equal(Vector2::Zero(), exactFactor.evaluateError(kTw)));

  Values values;
  values.insert(kKey, kTw);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

}  // namespace pose2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace pose3_fixture {

const Key kKey = 4;
const Pose3 kTw(Rot3::Expmap(Vector3(0.2, -0.1, 0.3)), Point3(0.4, -0.2, 0.6));
const Point3 wP(1.2, -0.7, 0.5);
const Point3 measured_kP(0.3, 0.8, -0.4);

// Verifies the Pose3 residual, exact correspondence, and analytic Jacobian.
TEST(KnownLandmarkFactor, Pose3) {
  const auto model = noiseModel::Unit::Create(3);
  const KnownLandmarkFactor<Pose3> factor(kKey, wP, measured_kP, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTw, H);
  const Vector expected = kTw.transformFrom(wP) - measured_kP;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(6, H.cols());

  const KnownLandmarkFactor<Pose3> exactFactor(kKey, wP, kTw.transformFrom(wP),
                                               model);
  EXPECT(assert_equal(Vector3::Zero(), exactFactor.evaluateError(kTw)));

  Values values;
  values.insert(kKey, kTw);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies that a full information matrix determines the factor cost.
TEST(KnownLandmarkFactor, FullInformationMatrix) {
  Matrix3 information;
  information << 4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0;
  const auto model = noiseModel::Gaussian::Information(information);
  const KnownLandmarkFactor<Pose3> factor(kKey, wP, measured_kP, model);

  Values values;
  values.insert(kKey, kTw);
  const Vector error = factor.evaluateError(kTw);
  const double expected = 0.5 * error.dot(information * error);
  EXPECT_DOUBLES_EQUAL(expected, factor.error(values), 1e-9);
}

}  // namespace pose3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace matrix_weighted_localization_fixture {

// Verifies that the complete localization graph and its D=1 QCQP have the
// same nonzero objective at a common Pose3 assignment.
TEST(KnownLandmarkFactor, MatrixWeightedLocalizationQcqpError) {
  const auto matrixWeightedProblem = examples::loadMatrixWeightedLocalization(
      findExampleDataFile("matrix_weighted_localization.g2o"));

  const QcqpProblem qcqp(matrixWeightedProblem.graph, 1);
  Values qcqpValues;
  for (const Key k : matrixWeightedProblem.poseKeys) {
    InsertQcqpValue<Pose3, 1>(
        k, matrixWeightedProblem.initial_kTws.at<Pose3>(k), &qcqpValues);
  }

  const double nonlinearError =
      matrixWeightedProblem.graph.error(matrixWeightedProblem.initial_kTws);
  EXPECT(nonlinearError > 0.0);
  EXPECT_DOUBLES_EQUAL(nonlinearError, qcqp.costs().error(qcqpValues), 1e-9);
}

}  // namespace matrix_weighted_localization_fixture
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
