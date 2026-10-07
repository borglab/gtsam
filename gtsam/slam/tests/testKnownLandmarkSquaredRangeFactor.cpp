/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testKnownLandmarkFactor.cpp
 * @brief Tests for KnownLandmarkSquaredRangeFactor
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <examples/CertifiableLocalizationUtils.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/KnownLandmarkSquaredRangeFactor.h>

using namespace gtsam;

/* ************************************************************************* */
namespace pose2_fixture {

const Key kKey = 3;
const Point2 wX(0.4, -0.2, 0.3);
const Point2 wL(1.2, -0.7);
const Point2 measured_kP(0.3, 0.8);

// Verifies the Pose2 residual, exact correspondence, and analytic Jacobian.
TEST(KnownLandmarkSquaredRangeFactor, Point2) {
  const auto model = noiseModel::Unit::Create(2);
  const KnownLandmarkFactor<Pose2> factor(kKey, wL, measured_kP, model);

  Matrix H;
  const Vector actual = factor.evaluateError(wTk, H);
  const Vector expected = wTk.transformTo(wL) - measured_kP;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(2, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const KnownLandmarkFactor<Pose2> exactFactor(kKey, wL, wTk.transformTo(wL),
                                               model);
  EXPECT(assert_equal(Vector2::Zero(), exactFactor.evaluateError(wTk)));

  Values values;
  values.insert(kKey, wTk);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

}  // namespace pose2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace pose3_fixture {

const Key kKey = 4;
const Pose3 wTk(Rot3::Expmap(Vector3(0.2, -0.1, 0.3)), Point3(0.4, -0.2, 0.6));
const Point3 wL(1.2, -0.7, 0.5);
const Point3 measured_kP(0.3, 0.8, -0.4);

// Verifies the Pose3 residual, exact correspondence, and analytic Jacobian.
TEST(KnownLandmarkFactor, Pose3) {
  const auto model = noiseModel::Unit::Create(3);
  const KnownLandmarkFactor<Pose3> factor(kKey, wL, measured_kP, model);

  Matrix H;
  const Vector actual = factor.evaluateError(wTk, H);
  const Vector expected = wTk.transformTo(wL) - measured_kP;
  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(6, H.cols());

  const KnownLandmarkFactor<Pose3> exactFactor(kKey, wL, wTk.transformTo(wL),
                                               model);
  EXPECT(assert_equal(Vector3::Zero(), exactFactor.evaluateError(wTk)));

  Values values;
  values.insert(kKey, wTk);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies that a full information matrix determines the factor cost.
TEST(KnownLandmarkFactor, FullInformationMatrix) {
  Matrix3 information;
  information << 4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0;
  const auto model = noiseModel::Gaussian::Information(information);
  const KnownLandmarkFactor<Pose3> factor(kKey, wL, measured_kP, model);

  Values values;
  values.insert(kKey, wTk);
  const Vector error = factor.evaluateError(wTk);
  const double expected = 0.5 * error.dot(information * error);
  EXPECT_DOUBLES_EQUAL(expected, factor.error(values), 1e-9);
}

// Verifies polymorphic cloning and measurement-aware equality.
TEST(KnownLandmarkFactor, CloneAndEquals) {
  const auto model = noiseModel::Unit::Create(3);
  const KnownLandmarkFactor<Pose3> factor(kKey, wL, measured_kP, model);
  const auto clone = factor.clone();
  EXPECT(factor.equals(*clone));
  EXPECT(!factor.equals(KnownLandmarkFactor<Pose3>(
      kKey, wL + Point3(0.1, 0.0, 0.0), measured_kP, model)));
  EXPECT(!factor.equals(KnownLandmarkFactor<Pose3>(
      kKey, wL, measured_kP + Point3(0.0, 0.1, 0.0), model)));

  NonlinearFactorGraph graph;
  graph.push_back(clone);
  const NonlinearFactorGraph graphClone = graph.clone();
  EXPECT(std::dynamic_pointer_cast<KnownLandmarkFactor<Pose3>>(
      graphClone.at(0)));
}

}  // namespace pose3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
