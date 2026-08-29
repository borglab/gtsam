/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactorWithGravity.cpp
 * @brief   Unit test for the gravity-aware ImuFactor variants
 * @author  Nikhil Khedekar
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CombinedImuFactor.h>  // for TEST_PIM's CombinedPIM
#include <gtsam/navigation/ImuFactorWithGravity.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/VectorNormFactor.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "imuFactorTesting.h"

/* ************************************************************************* */
// Common linearization point and measurements, as in testImuFactor.cpp
namespace common {
static const Pose3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0),
    Point3(5.0, 1.0, 0));
static const Vector3 v1(Vector3(0.5, 0.0, 0.0));

// Measurements
static const double w = M_PI / 100;
static const Vector3 measuredOmega(w, 0, 0);
static const Vector3 measuredAcc = x1.rotation().unrotate(
    -kGravityAlongNavZDown);
static const double deltaT = 1.0;

static const Pose3 x2(Rot3::RzRyRx(M_PI / 12.0 + w, M_PI / 6.0, M_PI / 4.0),
    Point3(5.5, 1.0, 0));
static const Vector3 v2(Vector3(0.5, 0.0, 0.0));
} // namespace common
/* ************************************************************************* */

/* ************************************************************************* */
TEST_PIM(ImuFactorWithGravity, DirectionJacobians) {
  using namespace common;
  using symbol_shorthand::G;
  auto params = testing::Params();
  params->omegaCoriolis = kNonZeroOmegaCoriolis;
  PIM pim(params);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  ImuFactorWithGravityT<PIM, Unit3> factor(X(1), V(1), X(2), V(2), B(1), G(0),
                                           pim);
  // The default magnitude comes from the params' gravity vector:
  DOUBLES_EQUAL(testing::Params()->n_gravity.norm(), factor.gravityMagnitude(),
                1e-9);

  // With the params' gravity direction, the error must match plain ImuFactor:
  ImuFactorT<PIM> plain(X(1), V(1), X(2), V(2), B(1), pim);
  EXPECT(assert_equal(
      plain.evaluateError(x1, v1, x2, v2, kZeroBias),
      factor.evaluateError(x1, v1, x2, v2, kZeroBias,
                           Unit3(testing::Params()->n_gravity))));

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), kZeroBias);
  values.insert(G(0), Unit3(0.1, -0.2, -1.0));  // tilted away from params
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  // The magnitude scales the gravity Jacobian block; check it at a
  // deliberately non-default value:
  ImuFactorWithGravityT<PIM, Unit3> lunar(X(1), V(1), X(2), V(2), B(1), G(0),
                                          pim, 1.62);
  EXPECT_CORRECT_FACTOR_JACOBIANS(lunar, values, 1e-7, 1e-3);
}

/* ************************************************************************* */
TEST_PIM(ImuFactorWithGravity, VectorJacobians) {
  using namespace common;
  using symbol_shorthand::G;
  auto params = testing::Params();
  params->omegaCoriolis = kNonZeroOmegaCoriolis;
  PIM pim(params);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  ImuFactorWithGravityT<PIM, Point3> factor(X(1), V(1), X(2), V(2), B(1), G(0),
                                            pim);

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), kZeroBias);
  // Tilted direction and non-standard magnitude:
  values.insert(G(0), Point3(0.4, -0.6, -9.5));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

/* ************************************************************************* */
TEST(ImuFactorWithGravity, Equals) {
  using symbol_shorthand::G;
  auto pim = PreintegratedImuMeasurements(testing::Params());
  ImuFactorWithGravityDirection factor1(X(1), V(1), X(2), V(2), B(1), G(0), pim);
  ImuFactorWithGravityDirection factor2(X(1), V(1), X(2), V(2), B(1), G(0), pim);
  ImuFactorWithGravityDirection factor3(X(1), V(1), X(2), V(2), B(1), G(0), pim,
                                        1.62);  // lunar gravity
  EXPECT(factor1.equals(factor2));
  EXPECT(!factor1.equals(factor3));
  // Comparing against a different factor type must not crash:
  ImuFactorWithGravityVector vectorFactor(X(1), V(1), X(2), V(2), B(1), G(0), pim);
  EXPECT(!factor1.equals(vectorFactor));
  EXPECT(!vectorFactor.equals(factor1));
}

/* ************************************************************************* */
TEST(ImuFactorWithGravity, ConstructorValidation) {
  using symbol_shorthand::G;
  auto pim = PreintegratedImuMeasurements(testing::Params());
  // The Unit3 parametrization requires a positive magnitude:
  CHECK_EXCEPTION(ImuFactorWithGravityDirection(X(1), V(1), X(2), V(2), B(1),
                                                G(0), pim, 0.0),
                  std::invalid_argument);
  // The Point3 parametrization optimizes the magnitude as part of the gravity
  // variable, so providing one is rejected (use VectorNormFactor<3> instead):
  CHECK_EXCEPTION(ImuFactorWithGravityVector(X(1), V(1), X(2), V(2), B(1),
                                             G(0), pim, 9.81),
                  std::invalid_argument);
  // Without params there is no default magnitude for the Unit3 parametrization
  // to fall back on; an explicit one still works, and Point3 needs none:
  const PreintegratedImuMeasurements noParams;
  CHECK_EXCEPTION(ImuFactorWithGravityDirection(X(1), V(1), X(2), V(2), B(1),
                                                G(0), noParams),
                  std::invalid_argument);
  const ImuFactorWithGravityDirection explicitMagnitude(
      X(1), V(1), X(2), V(2), B(1), G(0), noParams, 9.81);
  DOUBLES_EQUAL(9.81, explicitMagnitude.gravityMagnitude(), 1e-9);
  const ImuFactorWithGravityVector vectorNoParams(X(1), V(1), X(2), V(2), B(1),
                                                  G(0), noParams);
  EXPECT_LONGS_EQUAL(6, vectorNoParams.size());
}

/* ************************************************************************* */
// A stationary IMU in a nav frame whose gravity is tilted away from the
// params' -z direction: the accelerometer measures -g in the body frame, so
// optimizing the gravity variable must recover the true tilted gravity.
namespace tilted {
static const Vector3 trueGravity =
    Rot3::Rodrigues(0.05, -0.03, 0.0) * Vector3(0, 0, -9.81);
static PreintegratedImuMeasurements integrateStationary() {
  auto p = testing::Params();  // note: n_gravity = (0, 0, kGravity) = (0, 0, 10)
  PreintegratedImuMeasurements pim(p);
  for (int i = 0; i < 10; ++i)
    pim.integrateMeasurement(-trueGravity, Vector3::Zero(), 0.1);
  return pim;
}
static void addStationaryPriors(NonlinearFactorGraph* graph) {
  using symbol_shorthand::G;
  auto tightPose = noiseModel::Isotropic::Sigma(6, 1e-6);
  auto tightVector = noiseModel::Isotropic::Sigma(3, 1e-6);
  auto tightBias = noiseModel::Isotropic::Sigma(6, 1e-6);
  graph->addPrior(X(1), Pose3::Identity(), tightPose);
  graph->addPrior(X(2), Pose3::Identity(), tightPose);
  graph->addPrior(V(1), Vector3(Vector3::Zero()), tightVector);
  graph->addPrior(V(2), Vector3(Vector3::Zero()), tightVector);
  graph->addPrior(B(1), kZeroBias, tightBias);
}
static Values stationaryInitial() {
  Values initial;
  initial.insert(X(1), Pose3::Identity());
  initial.insert(X(2), Pose3::Identity());
  initial.insert(V(1), Vector3(Vector3::Zero()));
  initial.insert(V(2), Vector3(Vector3::Zero()));
  initial.insert(B(1), kZeroBias);
  return initial;
}
}  // namespace tilted

TEST(ImuFactorWithGravity, RecoverGravityDirection) {
  using symbol_shorthand::G;
  NonlinearFactorGraph graph;
  // The magnitude is given explicitly since the params' norm is kGravity = 10,
  // while the data was generated with |g| = 9.81:
  graph.emplace_shared<ImuFactorWithGravityDirection>(
      X(1), V(1), X(2), V(2), B(1), G(0), tilted::integrateStationary(), 9.81);
  tilted::addStationaryPriors(&graph);

  Values initial = tilted::stationaryInitial();
  initial.insert(G(0), Unit3(0, 0, -1));

  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  EXPECT(assert_equal(Unit3(tilted::trueGravity), result.at<Unit3>(G(0)), 1e-5));
}

/* ************************************************************************* */
TEST(ImuFactorWithGravity, RecoverGravityVector) {
  using symbol_shorthand::G;
  NonlinearFactorGraph graph;
  graph.emplace_shared<ImuFactorWithGravityVector>(
      X(1), V(1), X(2), V(2), B(1), G(0), tilted::integrateStationary());
  // Lupton-style magnitude pseudo-observation (consistent with trueGravity):
  graph.emplace_shared<VectorNormFactor<3>>(
      G(0), 9.81, noiseModel::Isotropic::Sigma(1, 0.03));
  tilted::addStationaryPriors(&graph);

  Values initial = tilted::stationaryInitial();
  initial.insert(G(0), Point3(0, 0, -9.0));

  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  EXPECT(assert_equal(Point3(tilted::trueGravity), result.at<Point3>(G(0)), 1e-4));
}

/* ************************************************************************* */
#if defined(GTSAM_TANGENT_PREINTEGRATION) && \
    !defined(GTSAM_LIEGROUP_PREINTEGRATION)
TEST(ImuFactorWithGravity, Merge) {
  using symbol_shorthand::G;
  auto p = testing::Params();
  PreintegratedImuMeasurements pim01(p), pim12(p);
  const Vector3 acc(0.1, 0.2, -9.81), omega(0.1, 0.02, 0.03);
  for (int i = 0; i < 10; ++i) {
    pim01.integrateMeasurement(acc, omega, 0.1);
    pim12.integrateMeasurement(acc, omega, 0.1);
  }
  auto f01 = std::make_shared<ImuFactorWithGravityDirection>(
      X(1), V(1), X(2), V(2), B(1), G(0), pim01);
  auto f12 = std::make_shared<ImuFactorWithGravityDirection>(
      X(2), V(2), X(3), V(3), B(1), G(0), pim12);

  auto f02 = ImuFactorWithGravityDirection::Merge(f01, f12);
  DOUBLES_EQUAL(2.0, f02->preintegratedMeasurements().deltaTij(), 1e-9);
  DOUBLES_EQUAL(f01->gravityMagnitude(), f02->gravityMagnitude(), 1e-9);
  EXPECT(f02->key<1>() == X(1));
  EXPECT(f02->key<3>() == X(3));
  EXPECT(f02->key<6>() == G(0));

  // Mismatched gravity magnitudes must be rejected:
  auto f12_moon = std::make_shared<ImuFactorWithGravityDirection>(
      X(2), V(2), X(3), V(3), B(1), G(0), pim12, 1.62);
  CHECK_EXCEPTION(ImuFactorWithGravityDirection::Merge(f01, f12_moon),
                  std::domain_error);

  // Mismatched gravity keys must be rejected:
  auto f12_key = std::make_shared<ImuFactorWithGravityDirection>(
      X(2), V(2), X(3), V(3), B(1), G(1), pim12);
  CHECK_EXCEPTION(ImuFactorWithGravityDirection::Merge(f01, f12_key),
                  std::domain_error);

  // The Point3 parametrization merges without a magnitude:
  auto v01 = std::make_shared<ImuFactorWithGravityVector>(
      X(1), V(1), X(2), V(2), B(1), G(0), pim01);
  auto v12 = std::make_shared<ImuFactorWithGravityVector>(
      X(2), V(2), X(3), V(3), B(1), G(0), pim12);
  auto v02 = ImuFactorWithGravityVector::Merge(v01, v12);
  DOUBLES_EQUAL(2.0, v02->preintegratedMeasurements().deltaTij(), 1e-9);
}
#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
