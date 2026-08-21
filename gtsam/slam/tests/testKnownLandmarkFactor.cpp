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
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/KnownLandmarkFactor.h>

#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace rot2_fixture {

const Key kKey = 1;
const Rot2 kTransform = Rot2::fromAngle(0.4);
const Point2 kSourcePoint(1.2, -0.7);
const Point2 kMeasuredPoint(0.3, 0.8);

// Verifies the Rot2 residual, exact correspondence, and analytic Jacobian.
TEST(KnownLandmarkFactor, Rot2) {
  const auto model = noiseModel::Unit::Create(2);
  const KnownLandmarkFactor<Rot2> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected = kTransform.rotate(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(2, H.rows());
  EXPECT_LONGS_EQUAL(1, H.cols());

  const KnownLandmarkFactor<Rot2> exactFactor(
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
TEST(KnownLandmarkFactor, Rot3) {
  const auto model = noiseModel::Unit::Create(3);
  const KnownLandmarkFactor<Rot3> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected = kTransform.rotate(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const KnownLandmarkFactor<Rot3> exactFactor(
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
TEST(KnownLandmarkFactor, Pose2) {
  const auto model = noiseModel::Unit::Create(2);
  const KnownLandmarkFactor<Pose2> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected =
      kTransform.transformFrom(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(2, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const KnownLandmarkFactor<Pose2> exactFactor(
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
TEST(KnownLandmarkFactor, Pose3) {
  const auto model = noiseModel::Unit::Create(3);
  const KnownLandmarkFactor<Pose3> factor(
      kKey, kSourcePoint, kMeasuredPoint, model);

  Matrix H;
  const Vector actual = factor.evaluateError(kTransform, H);
  const Vector expected =
      kTransform.transformFrom(kSourcePoint) - kMeasuredPoint;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(6, H.cols());

  const KnownLandmarkFactor<Pose3> exactFactor(
      kKey, kSourcePoint, kTransform.transformFrom(kSourcePoint), model);
  EXPECT(assert_equal(Vector3::Zero(), exactFactor.evaluateError(kTransform)));

  Values values;
  values.insert(kKey, kTransform);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies that a full information matrix determines the factor cost.
TEST(KnownLandmarkFactor, FullInformationMatrix) {
  Matrix3 information;
  information << 4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0;
  const auto model = noiseModel::Gaussian::Information(information);
  const KnownLandmarkFactor<Pose3> factor(
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
namespace matrix_weighted_localization_fixture {

// Verifies that the complete localization graph and its D=1 QCQP have the
// same nonzero objective at a common Pose3 assignment.
TEST(KnownLandmarkFactor, MatrixWeightedLocalizationQcqpError) {
  constexpr size_t kNumPoses = 3;
  constexpr double kRadialSigma = 0.1;
  constexpr double kLateralSigma = 0.01;
  constexpr double kOdometrySigma = 0.1;

  const Pose3 step(Rot3::RzRyRx(0.03, -0.05, 0.08),
                   Point3(0.3, -0.1, 0.2));
  std::vector<Pose3> groundTruth{Pose3()};
  for (size_t i = 1; i < kNumPoses; ++i) {
    groundTruth.push_back(groundTruth.back().compose(step));
  }

  const std::vector<Point3> landmarks{
      Point3(4.0, 1.0, 2.0), Point3(-1.0, 3.0, 5.0),
      Point3(2.0, -4.0, 3.0), Point3(5.0, 2.0, -1.0)};

  NonlinearFactorGraph graph;
  const double radialPrecision = 1.0 / (kRadialSigma * kRadialSigma);
  const double lateralPrecision = 1.0 / (kLateralSigma * kLateralSigma);
  for (size_t k = 0; k < groundTruth.size(); ++k) {
    for (const Point3& landmark : landmarks) {
      const Point3 measurement = groundTruth[k].transformFrom(landmark);
      const Vector3 ray = measurement.normalized();
      const Matrix3 information =
          lateralPrecision * Matrix3::Identity() +
          (radialPrecision - lateralPrecision) * ray * ray.transpose();
      graph.emplace_shared<KnownLandmarkFactor<Pose3>>(
          k, landmark, measurement,
          noiseModel::Gaussian::Information(information));
    }
  }

  const auto odometryNoiseModel =
      noiseModel::Isotropic::Sigma(Pose3::dimension, kOdometrySigma);
  for (size_t i = 0; i + 1 < groundTruth.size(); ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Pose3>>(
        i, i + 1, groundTruth[i].between(groundTruth[i + 1]),
        odometryNoiseModel);
  }

  Values values;
  Vector6 perturbation;
  perturbation << 0.02, -0.015, 0.01, 0.08, -0.05, 0.06;
  for (size_t k = 0; k < groundTruth.size(); ++k) {
    values.insert(
        k, groundTruth[k].retract(static_cast<double>(k + 1) * perturbation));
  }

  const QcqpProblem qcqp(graph, 1);
  Values qcqpValues;
  for (size_t k = 0; k < groundTruth.size(); ++k) {
    InsertQcqpValue<Pose3, 1>(k, values.at<Pose3>(k), &qcqpValues);
  }

  const double nonlinearError = graph.error(values);
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
