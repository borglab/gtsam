/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactor.cpp
 * @brief   Unit test for ImuFactor
 * @author  Luca Carlone
 * @author  Frank Dellaert
 * @author  Richard Roberts
 * @author  Stephen Williams
 */

// #define ENABLE_TIMING // uncomment for timing results

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/VectorConstants.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <list>

#include "imuFactorTesting.h"

namespace default_backend {

#ifdef GTSAM_LIEGROUP_PREINTEGRATION
static_assert(
    std::is_same<DefaultPreintegrationType, LieGroupPreintegration>::value,
    "Lie-group preintegration must take precedence as the default backend");
#elif defined(GTSAM_TANGENT_PREINTEGRATION)
static_assert(
    std::is_same<DefaultPreintegrationType, TangentPreintegration>::value,
    "Tangent preintegration must remain the default backend");
#else
static_assert(
    std::is_same<DefaultPreintegrationType, ManifoldPreintegration>::value,
    "Manifold preintegration must be selected when both options are disabled");
#endif

}  // namespace default_backend

/* ************************************************************************* */
TEST_PIM(ImuFactor, PreintegratedMeasurementsConstruction) {
  // Actual pre-integrated values
  auto params = testing::Params();
  params->omegaCoriolis = kNonZeroOmegaCoriolis;
  PIM actual(params);
  EXPECT(assert_equal(Rot3(), actual.deltaRij()));
  EXPECT(assert_equal(kZero, actual.deltaPij()));
  EXPECT(assert_equal(kZero, actual.deltaVij()));
  DOUBLES_EQUAL(0.0, actual.deltaTij(), 1e-9);
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, PreintegratedMeasurementsReset) {

	auto p = testing::Params();
	// Create a preintegrated measurement struct and integrate
	PIM pimActual(p);
	Vector3 measuredAcc(0.5, 1.0, 0.5);
	Vector3 measuredOmega(0.1, 0.3, 0.1);
	double deltaT = 1.0;
	pimActual.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

	// reset and make sure that it is the same as a fresh one
	pimActual.resetIntegration();
	CHECK(assert_equal(pimActual, PIM(p)));

	// Now create one with a different bias ..
	Bias nonZeroBias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3));
	PIM pimExpected(p, nonZeroBias);

	// integrate again, then reset to a new bias
	pimActual.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
	pimActual.resetIntegrationAndSetBias(nonZeroBias);
	CHECK(assert_equal(pimActual, pimExpected));
}

/* ************************************************************************* */
// Checks propagated sensor-noise covariance against sampled NavState errors.
TEST(ImuFactor, Accelerating) {
  const double a = 0.2, v = 50;

  // Set up body pointing towards y axis, and start at 10,20,0 with velocity going in X
  // The body itself has Z axis pointing down
  const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
  const Point3 initial_position(10, 20, 0);
  const Vector3 initial_velocity(v, 0, 0);

  const AcceleratingScenario scenario(nRb, initial_position, initial_velocity,
      Vector3(a, 0, 0));

  const double T = 3.0;  // seconds
  auto params = testing::Params();
  // ScenarioRunner samples accelerometer and gyroscope noise, but not the
  // independent position-integration uncertainty.
  params->integrationCovariance = Z_3x3;
  ScenarioRunner runner(scenario, params, T / 10);

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  const Matrix9 estimatedCov = runner.estimateCovariance(T, 5000);
  const Matrix9 expectedCov = pim.residualCovariance();
  const double relativeError =
      (estimatedCov - expectedCov).norm() / expectedCov.norm();
  EXPECT(relativeError < 0.12);
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, PreintegratedMeasurements) {
  // Measurements
  const double a = 0.1, w = M_PI / 100.0;
  Vector3 measuredAcc(a, 0.0, 0.0);
  Vector3 measuredOmega(w, 0.0, 0.0);
  double deltaT = 0.5;

  // Expected pre-integrated values
  Vector3 expectedDeltaR1(w * deltaT, 0.0, 0.0);
  Vector3 expectedDeltaP1(0.5 * a * deltaT*deltaT, 0, 0);
  Vector3 expectedDeltaV1(0.05, 0.0, 0.0);

  // Actual pre-integrated values
  PIM actual(testing::Params());
  actual.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  EXPECT(assert_equal(Rot3::Expmap(expectedDeltaR1), actual.deltaRij()));
  EXPECT(assert_equal(expectedDeltaP1, actual.deltaPij()));
  EXPECT(assert_equal(expectedDeltaV1, actual.deltaVij()));
  DOUBLES_EQUAL(0.5, actual.deltaTij(), 1e-9);

  // Check factor derivatives rather than exposing residual assembly on the PIM.
  Bias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  NavState x1, x2 = actual.predict(x1, bias);
  ImuFactor2T<PIM> factor(X(1), X(2), B(1), actual);
  Values values;
  values.insert(X(1), x1);
  values.insert(X(2), x2);
  values.insert(B(1), bias);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);

  // Integrate again
  Vector3 expectedDeltaR2(2.0 * 0.5 * M_PI / 100.0, 0.0, 0.0);
  Vector3 expectedDeltaP2(0.025 + expectedDeltaP1(0) + 0.5 * 0.1 * 0.5 * 0.5, 0, 0);
  Vector3 expectedDeltaV2 = Vector3(0.05, 0.0, 0.0) +
                            Rot3::Expmap(expectedDeltaR1) * measuredAcc * 0.5;

  // Actual pre-integrated values
  actual.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  EXPECT(assert_equal(Rot3::Expmap(expectedDeltaR2), actual.deltaRij()));
  EXPECT(assert_equal(expectedDeltaP2, actual.deltaPij()));
  EXPECT(assert_equal(expectedDeltaV2, actual.deltaVij()));
  DOUBLES_EQUAL(1.0, actual.deltaTij(), 1e-9);
}

namespace deskew {

Vector3 pointAt(const Matrix& points, Eigen::Index pointIndex,
                Eigen::Index batchIndex) {
  return points.block<3, 1>(3 * pointIndex, batchIndex);
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, DeskewImplicitAndExplicitTiming) {
  PIM pim(testing::Params());
  const double yawRate = 0.4, duration = 2.0;
  pim.integrateMeasurement(Z_3x1, Vector3(0.0, 0.0, yawRate), duration);

  const Vector3 first(1.0, 0.0, 0.0), second(0.0, 1.0, 0.5);
  Matrix points(6, 4);
  for (Eigen::Index batch = 0; batch < points.cols(); ++batch) {
    points.block<3, 1>(0, batch) = first;
    points.block<3, 1>(3, batch) = second;
  }

  const Matrix implicit = pim.deskewPoints(points);
  for (Eigen::Index batch = 0; batch < implicit.cols(); ++batch) {
    const double t = duration * static_cast<double>(batch) / implicit.cols();
    const Rot3 rotation = Rot3::Yaw(yawRate * t);
    EXPECT(assert_equal(rotation.rotate(first), pointAt(implicit, 0, batch),
                        1e-8));
    EXPECT(assert_equal(rotation.rotate(second), pointAt(implicit, 1, batch),
                        1e-8));
  }
  CHECK((pointAt(implicit, 0, 3) - pim.deltaRij().rotate(first)).norm() >
        1e-3);

  const Vector3 velocity(0.5, -0.2, 0.1);
  const Matrix implicitWithVelocity = pim.deskewPoints(points, velocity);
  for (Eigen::Index batch = 0; batch < implicitWithVelocity.cols(); ++batch) {
    const double t =
        duration * static_cast<double>(batch) / implicitWithVelocity.cols();
    const Rot3 rotation = Rot3::Yaw(yawRate * t);
    const Vector3 expectedFirst = rotation.rotate(first) + velocity * t;
    const Vector3 expectedSecond = rotation.rotate(second) + velocity * t;
    EXPECT(assert_equal(expectedFirst,
                        pointAt(implicitWithVelocity, 0, batch), 1e-8));
    EXPECT(assert_equal(expectedSecond,
                        pointAt(implicitWithVelocity, 1, batch), 1e-8));
  }

  Vector times(4);
  times << duration, 0.0, 0.5 * duration, 0.25 * duration;
  const Matrix explicitResult = pim.deskewPointsAtTimes(points, times);
  for (Eigen::Index batch = 0; batch < explicitResult.cols(); ++batch) {
    const Rot3 rotation = Rot3::Yaw(yawRate * times(batch));
    EXPECT(assert_equal(rotation.rotate(first),
                        pointAt(explicitResult, 0, batch), 1e-8));
    EXPECT(assert_equal(rotation.rotate(second),
                        pointAt(explicitResult, 1, batch), 1e-8));
  }
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, DeskewVelocityAndValidation) {
  PIM pim(testing::Params());
  pim.integrateMeasurement(Z_3x1, Vector3(0.0, 0.0, 0.4), 2.0);

  Matrix points(6, 2);
  points << 1.0, -0.5, 0.0, 0.2, 0.0, 1.0, -0.2, 1.5, 0.8, 0.3, 1.2,
      -0.4;
  Vector times(2);
  times << 2.0, 0.5;
  const Vector3 velocity(0.5, -0.2, 0.1);
  const Matrix actual = pim.deskewPointsAtTimes(points, times, velocity);
  for (Eigen::Index batch = 0; batch < actual.cols(); ++batch) {
    const Pose3 transform(Rot3::Expmap(pim.so3TangentAt(times(batch))),
                          velocity * times(batch));
    for (Eigen::Index pointIndex = 0; pointIndex < 2; ++pointIndex) {
      EXPECT(assert_equal(
          transform.transformFrom(pointAt(points, pointIndex, batch)),
          pointAt(actual, pointIndex, batch), 1e-8));
    }
  }

  const Matrix empty(0, 3);
  EXPECT(assert_equal(empty, pim.deskewPoints(empty)));
  CHECK_EXCEPTION(pim.deskewPoints(Matrix::Zero(2, 2)),
                  std::invalid_argument);
  CHECK_EXCEPTION(pim.deskewPoints(Matrix::Zero(4, 3)),
                  std::invalid_argument);
  CHECK_EXCEPTION(pim.deskewPointsAtTimes(Matrix::Zero(3, 2), Vector::Zero(1)),
                  std::invalid_argument);
  Vector badTime(1);
  badTime << -1e-6;
  CHECK_EXCEPTION(pim.deskewPointsAtTimes(Matrix::Zero(3, 1), badTime),
                  std::out_of_range);
  badTime << pim.deltaTij() + 1e-6;
  CHECK_EXCEPTION(pim.deskewPointsAtTimes(Matrix::Zero(3, 1), badTime,
                                          Z_3x1),
                  std::out_of_range);
}

}  // namespace deskew

/* ************************************************************************* */
// Common linearization point and measurements for tests
namespace common {
static const Pose3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0),
    Point3(5.0, 1.0, 0));
static const Vector3 v1(Vector3(0.5, 0.0, 0.0));
static const NavState state1(x1, v1);

// Measurements
static const double w = M_PI / 100;
static const Vector3 measuredOmega(w, 0, 0);
static const Vector3 measuredAcc = x1.rotation().unrotate(
    -kGravityAlongNavZDown);
static const double deltaT = 1.0;

static const Pose3 x2(Rot3::RzRyRx(M_PI / 12.0 + w, M_PI / 6.0, M_PI / 4.0),
    Point3(5.5, 1.0, 0));
static const Vector3 v2(Vector3(0.5, 0.0, 0.0));
static const NavState state2(x2, v2);
} // namespace common

/* ************************************************************************* */
namespace fixed_linearization {

// Verifies fixed-size five-way IMU linearization changes only the factor type.
TEST_PIM(ImuFactor, FiveWayLinearizationIsBitwiseIdentical) {
  using namespace common;
  PIM pim(testing::Params());
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  const Key pose1Key = 101, velocity1Key = 102, pose2Key = 103;
  const Key velocity2Key = 104, biasKey = 105;
  const ImuFactorT<PIM> factor(pose1Key, velocity1Key, pose2Key, velocity2Key,
                               biasKey, pim);
  const Values values{{pose1Key, genericValue(x1)},
                      {velocity1Key, genericValue(v1)},
                      {pose2Key, genericValue(x2)},
                      {velocity2Key, genericValue(v2)},
                      {biasKey, genericValue(kZeroBias)}};
  const auto expectedBase = factor.NoiseModelFactor::linearize(values);
  const auto actualBase = factor.linearize(values);
  const auto expected = std::dynamic_pointer_cast<JacobianFactor>(expectedBase);
  const auto actual = std::dynamic_pointer_cast<JacobianFactor>(actualBase);

  const bool isFixed = static_cast<bool>(
      std::dynamic_pointer_cast<FixedJacobianFactor<9, 6, 3, 6, 3, 6>>(
          actualBase));
  CHECK(isFixed);
  CHECK(expected);
  CHECK(actual);
  CHECK(expected->keys() == actual->keys());
  CHECK(expected->get_model() == actual->get_model());
  CHECK((expected->getb().array() == actual->getb().array()).all());
  auto expectedBlock = expected->begin();
  auto actualBlock = actual->begin();
  for (; expectedBlock != expected->end(); ++expectedBlock, ++actualBlock) {
    CHECK((expected->getA(expectedBlock).array() ==
           actual->getA(actualBlock).array())
              .all());
  }
}

// Verifies fixed-size IMU linearization changes only the concrete factor type.
TEST_PIM(ImuFactor2, TernaryLinearizationIsBitwiseIdentical) {
  using namespace common;
  PIM pim(testing::Params());
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  const Key state1Key = 101, state2Key = 102, biasKey = 103;
  const ImuFactor2T<PIM> factor(state1Key, state2Key, biasKey, pim);
  const Values values{{state1Key, genericValue(state1)},
                      {state2Key, genericValue(state2)},
                      {biasKey, genericValue(kZeroBias)}};
  const auto expectedBase = factor.NoiseModelFactor::linearize(values);
  const auto actualBase = factor.linearize(values);
  const auto expected = std::dynamic_pointer_cast<JacobianFactor>(expectedBase);
  const auto actual = std::dynamic_pointer_cast<JacobianFactor>(actualBase);

  const bool isTernary = static_cast<bool>(
      std::dynamic_pointer_cast<FixedJacobianFactor<9, 9, 9, 6>>(actualBase));
  CHECK(isTernary);
  CHECK(expected);
  CHECK(actual);
  CHECK(expected->keys() == actual->keys());
  CHECK(expected->get_model() == actual->get_model());
  CHECK((expected->getb().array() == actual->getb().array()).all());
  auto expectedBlock = expected->begin();
  auto actualBlock = actual->begin();
  for (; expectedBlock != expected->end(); ++expectedBlock, ++actualBlock) {
    CHECK((expected->getA(expectedBlock).array() ==
           actual->getA(actualBlock).array())
              .all());
  }
}

}  // namespace fixed_linearization
/* ************************************************************************* */

/* ************************************************************************* */
// Checks the common prediction and bias-correction interface and Jacobians.
TEST_PIM(ImuFactor, PreintegrationBaseMethods) {
  // Select the overload without the gravity parameter:
  using PredictNoGravity = NavState (PreintegrationBase::*)(
      const NavState&, const imuBias::ConstantBias&, OptionalJacobian<9, 9>,
      OptionalJacobian<9, 6>) const;
  using namespace common;
  auto p = testing::Params();
  p->omegaCoriolis = Vector3(0.02, 0.03, 0.04);

  PIM pim(p, kZeroBiasHat);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // biasCorrectedDelta
  Matrix96 actualH;
  pim.biasCorrectedDelta(kZeroBias, actualH);
  Matrix expectedH = numericalDerivative11<Vector9, Bias>(
      std::bind(&PreintegrationBase::biasCorrectedDelta, pim,
                std::placeholders::_1, nullptr),
      kZeroBias);
  constexpr double derivativeTolerance =
      std::is_same_v<PIM, PreintegratedImuMeasurementsG> ? 1e-8 : 1e-9;
  EXPECT(assert_equal(expectedH, actualH, derivativeTolerance));

  Matrix9 aH1;
  Matrix96 aH2;
  NavState predictedState = pim.predict(state1, kZeroBias, aH1, aH2);
  Matrix eH1 = numericalDerivative11<NavState, NavState>(
      std::bind(static_cast<PredictNoGravity>(&PreintegrationBase::predict),
                pim, std::placeholders::_1, kZeroBias, nullptr, nullptr),
      state1);
  EXPECT(assert_equal(eH1, aH1));
  Matrix eH2 = numericalDerivative11<NavState, Bias>(
      std::bind(static_cast<PredictNoGravity>(&PreintegrationBase::predict),
                pim, state1, std::placeholders::_1, nullptr, nullptr),
      kZeroBias);
  EXPECT(assert_equal(eH2, aH2, derivativeTolerance));

  // Exercise the direct rotating-Earth prediction's gravity Jacobian.
  Matrix93 aH3;
  pim.predict(state1, kZeroBias, p->n_gravity, {}, {}, aH3);
  const Matrix eH3 = numericalDerivative11<NavState, Vector3>(
      [&](const Vector3& gravity) {
        return pim.predict(state1, kZeroBias, gravity);
      },
      p->n_gravity);
  EXPECT(assert_equal(eH3, Matrix(aH3)));
}

/* ************************************************************************* */
// Checks prediction with explicit gravity and all of its Jacobians.
TEST_PIM(ImuFactor, PredictWithGravityVector) {
  using namespace common;
  PIM pim(testing::Params(), kZeroBiasHat);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // A gravity vector tilted away from the params' vector, with a different
  // magnitude:
  const Vector3 tilted_gravity = Vector3(0.5, -0.3, 9.7);

  // The gravity overload with the params' gravity must match the legacy one:
  EXPECT(assert_equal(
      pim.predict(state1, kZeroBias),
      pim.predict(state1, kZeroBias, testing::Params()->n_gravity)));

  // Check all three Jacobians of the gravity overload:
  Matrix9 aH1;
  Matrix96 aH2;
  Matrix93 aH3;
  pim.predict(state1, kZeroBias, tilted_gravity, aH1, aH2, aH3);
  EXPECT(assert_equal(numericalDerivative11<NavState, NavState>(
                          [&](const NavState& s) {
                            return pim.predict(s, kZeroBias, tilted_gravity);
                          },
                          state1),
                      Matrix(aH1)));
  EXPECT(assert_equal(
      numericalDerivative11<NavState, Bias>(
          [&](const Bias& b) { return pim.predict(state1, b, tilted_gravity); },
          kZeroBias),
      Matrix(aH2),
      std::is_same_v<PIM, PreintegratedImuMeasurementsG> ? 1e-8 : 1e-9));
  EXPECT(assert_equal(
      numericalDerivative11<NavState, Vector3>(
          [&](const Vector3& g) { return pim.predict(state1, kZeroBias, g); },
          tilted_gravity),
      Matrix(aH3)));
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, MultipleMeasurements) {
  using namespace common;

  PIM expected(testing::Params(), kZeroBiasHat);
  expected.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  expected.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  Matrix32 acc,gyro;
  Matrix12 dts;
  acc << measuredAcc, measuredAcc;
  gyro << measuredOmega, measuredOmega;
  dts << deltaT, deltaT;
  PIM actual(testing::Params(), kZeroBiasHat);
  actual.integrateMeasurements(acc,gyro,dts);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Checks the factor residual and Jacobians at predicted and perturbed states.
TEST_PIM(ImuFactor, ErrorAndJacobians) {
  using namespace common;
  const auto params = testing::Params();
  PIM pim(params);

  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  NavState expectedState = state2;
  if constexpr (std::is_same_v<PIM, PreintegratedImuMeasurementsG>) {
    Vector10 input = Vector10::Zero();
    input.head<3>() = measuredOmega * deltaT;
    input.segment<3>(3) = measuredAcc * deltaT;
    input(9) = deltaT;
    const Gal3 delta = Gal3::Expmap(input);
    const Rot3 expectedRotation = x1.rotation().compose(delta.rotation());
    const Point3 expectedPosition = x1.translation() + v1 * deltaT +
                                    0.5 * params->n_gravity * deltaT * deltaT +
                                    x1.rotation().rotate(delta.translation());
    const Vector3 expectedVelocity = v1 + params->n_gravity * deltaT +
                                     x1.rotation().rotate(delta.velocity());
    expectedState =
        NavState(expectedRotation, expectedPosition, expectedVelocity);
  }
  EXPECT(assert_equal(expectedState, pim.predict(state1, kZeroBias)));

  const Pose3 expectedPose = expectedState.pose();
  const Vector3 expectedVelocity = expectedState.velocity();

  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Expected error
  Vector expectedError{{0, 0, 0, 0, 0, 0, 0, 0, 0}};
  EXPECT(assert_equal(
      expectedError,
      factor.evaluateError(x1, v1, expectedPose, expectedVelocity, kZeroBias)));

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), expectedPose);
  values.insert(V(2), expectedVelocity);
  values.insert(B(1), kZeroBias);
  EXPECT(assert_equal(expectedError, factor.unwhitenedError(values)));

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a;
  (void)factor.evaluateError(x1, v1, expectedPose, expectedVelocity, kZeroBias,
                             H1a, H2a, H3a, H4a, H5a);

  // Make sure rotation part is correct when error is interpreted as axis-angle
  // Jacobians are around zero, so the rotation part is the same as:
  Matrix H1Rot3 = numericalDerivative11<Rot3, Pose3>(
      [&](const Pose3& pose_i) {
        return Rot3::Expmap(factor
                                .evaluateError(pose_i, v1, expectedPose,
                                               expectedVelocity, kZeroBias)
                                .head(3));
      },
      x1);
  EXPECT(assert_equal(H1Rot3, H1a.topRows(3)));

  Matrix H3Rot3 = numericalDerivative11<Rot3, Pose3>(
      [&](const Pose3& pose_j) {
        return Rot3::Expmap(
            factor.evaluateError(x1, v1, pose_j, expectedVelocity, kZeroBias)
                .head(3));
      },
      expectedPose);
  EXPECT(assert_equal(H3Rot3, H3a.topRows(3)));

  // Evaluate error with wrong values
  Vector3 v2_wrong = expectedVelocity + Vector3(0.1, 0.1, 0.1);
  values.update(V(2), v2_wrong);
  expectedError << 0, 0, 0, 0, 0, 0, -0.0724744871, -0.040715657, -0.151952901;
  EXPECT(assert_equal(
      expectedError,
      factor.evaluateError(x1, v1, expectedPose, v2_wrong, kZeroBias), 1e-2));
  EXPECT(assert_equal(expectedError, factor.unwhitenedError(values), 1e-2));

  // Make sure the whitening is done correctly
  Matrix cov = pim.preintMeasCov();
  Eigen::LLT<Matrix> llt(cov.inverse());
  Matrix R = llt.matrixU();
  Vector whitened = R * expectedError;
  EXPECT(
      assert_equal(0.5 * whitened.squaredNorm(), factor.error(values), 1e-4));

  // Make sure linearization is correct
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, ErrorAndJacobianWithBiases) {
  using common::x1;
  using common::v1;
  using common::v2;
  Bias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 10.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));

  // Measurements
  Vector3 measuredOmega{0, 0, M_PI / 10.0 + 0.3};
  Vector3 measuredAcc = x1.rotation().unrotate(-kGravityAlongNavZDown)
      + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;

  auto p = testing::Params();
  p->omegaCoriolis = kNonZeroOmegaCoriolis;

  Bias biasHat(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1));
  PIM pim(p, biasHat);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Make sure of biasCorrectedDelta
  Matrix96 actualH;
  pim.biasCorrectedDelta(bias, actualH);
  Matrix expectedH = numericalDerivative11<Vector9, Bias>(
      std::bind(&PreintegrationBase::biasCorrectedDelta, pim,
          std::placeholders::_1, nullptr), bias);
  EXPECT(assert_equal(expectedH, actualH));

  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), bias);

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, ExactCoriolisIgnoresLegacySecondOrderFlag) {
  using common::x1;
  using common::v1;
  using common::v2;
  Bias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 10.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));

  // Measurements
  Vector3 measuredOmega{0, 0, M_PI / 10.0 + 0.3};
  Vector3 measuredAcc = x1.rotation().unrotate(-kGravityAlongNavZDown)
      + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;

  auto p = testing::Params();
  p->omegaCoriolis = kNonZeroOmegaCoriolis;
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  // The legacy flag is deliberately ignored by the exact Coriolis model.
  p->setUse2ndOrderCoriolis(true);
#endif

  PIM pim(p, Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)));
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  auto pWithoutLegacyFlag = testing::Params();
  pWithoutLegacyFlag->omegaCoriolis = kNonZeroOmegaCoriolis;
  PIM pimWithoutLegacyFlag(
      pWithoutLegacyFlag, Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)));
  pimWithoutLegacyFlag.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  EXPECT(assert_equal(pimWithoutLegacyFlag.predict(NavState(x1, v1), bias),
                      pim.predict(NavState(x1, v1), bias), 1e-12));

  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), bias);

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST(ImuFactor, PartialDerivative_wrt_Bias) {
  // Linearization point
  Vector3 biasOmega(0, 0, 0); // Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredOmega(0.1, 0, 0);
  double deltaT = 0.5;

  auto evaluateRotation = [&measuredOmega, &deltaT](const Vector3 biasOmega) {
    return Rot3::Expmap((measuredOmega - biasOmega) * deltaT);
  };

  // Compute numerical derivatives
  Matrix expectedDelRdelBiasOmega =
      numericalDerivative11<Rot3, Vector3>(evaluateRotation, biasOmega);

  const Matrix3 Jr =
      Rot3::ExpmapDerivative((measuredOmega - biasOmega) * deltaT);

  Matrix3 actualDelRdelBiasOmega = -Jr * deltaT; // the delta bias appears with the minus sign

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelRdelBiasOmega, actualDelRdelBiasOmega, 1e-9));
}

/* ************************************************************************* */
TEST(ImuFactor, PartialDerivativeLogmap) {
  // Linearization point
  Vector3 thetaHat(0.1, 0.1, 0); // Current estimate of rotation rate bias

  // Measurements
  Vector3 deltaTheta(0, 0, 0);

  auto evaluateLogRotation = [&thetaHat](const Vector3 delta) {
    return Rot3::Logmap(
        Rot3::Expmap(thetaHat).compose(Rot3::Expmap(delta)));
  };

  // Compute numerical derivatives
  Matrix expectedDelFdelTheta =
      numericalDerivative11<Vector, Vector3>(evaluateLogRotation, deltaTheta);

  Matrix3 actualDelFdelTheta = Rot3::LogmapDerivative(thetaHat);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelFdelTheta, actualDelFdelTheta));
}

/* ************************************************************************* */
TEST(ImuFactor, fistOrderExponential) {
  // Linearization point
  Vector3 biasOmega(0, 0, 0); // Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredOmega(0.1, 0, 0);
  double deltaT = 1.0;

  // change w.r.t. linearization point
  double alpha = 0.0;
  Vector3 deltaBiasOmega{alpha, alpha, alpha};

  const Matrix3 Jr = Rot3::ExpmapDerivative(
      (measuredOmega - biasOmega) * deltaT);

  Matrix3 delRdelBiasOmega = -Jr * deltaT; // the delta bias appears with the minus sign

  const Matrix expectedRot = Rot3::Expmap(
      (measuredOmega - biasOmega - deltaBiasOmega) * deltaT).matrix();

  const Matrix3 hatRot =
      Rot3::Expmap((measuredOmega - biasOmega) * deltaT).matrix();
  const Matrix3 actualRot = hatRot
      * Rot3::Expmap(delRdelBiasOmega * deltaBiasOmega).matrix();

  // This is a first order expansion so the equality is only an approximation
  EXPECT(assert_equal(expectedRot, actualRot));
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, ErrorWithBiasesAndSensorBodyDisplacement) {
  const Rot3 nRb = Rot3::Expmap(Vector3(0, 0, M_PI / 4.0));
  const Point3 p1(5.0, 1.0, -50.0);
  const Vector3 v1(0.5, 0.0, 0.0);

  const Vector3 a = nRb * Vector3(0.2, 0.0, 0.0);
  const AcceleratingScenario scenario(nRb, p1, v1, a,
      Vector3(0, 0, M_PI / 10.0 + 0.3));

  auto p = testing::Params();
  p->body_P_sensor = Pose3(Rot3::Expmap(Vector3(0, M_PI / 2, 0)),
      Point3(0.1, 0.05, 0.01));
  p->omegaCoriolis = kNonZeroOmegaCoriolis;

  Bias biasHat(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));

  const double T = 3.0; // seconds
  ScenarioRunner runner(scenario, p, T / 10);

  //  PIM pim = runner.integrate(T);
  //  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 1e-9));
  //
  //  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  //  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
  //
  ///////////////////////////////////////////////////////////////////////////////////////////
  Pose3 x1(nRb, p1);

  // Measurements
  Vector3 measuredOmega = runner.actualAngularVelocity(0);
  Vector3 measuredAcc = runner.actualSpecificForce(0);

  // Get mean prediction from "ground truth" measurements
  const Vector3 accNoiseVar2(0.01, 0.02, 0.03);
  const Vector3 omegaNoiseVar2(0.03, 0.01, 0.02);
  PIM pim(p, biasHat);

  // Check correctMeasurementsBySensorPose derivatives
  Matrix3 D_correctedAcc_measuredOmega = Z_3x3;
  pim.correctMeasurementsBySensorPose(measuredAcc, measuredOmega,
      nullptr, D_correctedAcc_measuredOmega, nullptr);
  auto correctedAcc = [&](const Vector3& measuredOmega) -> Vector3 {
    Vector3 correctedAcc = pim.biasHat().correctAccelerometer(measuredAcc);
    Vector3 correctedOmega = pim.biasHat().correctGyroscope(measuredOmega);
    return pim.correctMeasurementsBySensorPose(correctedAcc, correctedOmega).first;
  };
  Matrix3 expectedD = numericalDerivative11<Vector3, Vector3>(correctedAcc, measuredOmega, 1e-6);
  EXPECT(assert_equal(expectedD, D_correctedAcc_measuredOmega, 1e-5));

  // integrate at least twice to get position information
  // otherwise factor cov noise from preint_cov is not positive definite
  double dt = 0.1;
  pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
  pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
  
  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);
  
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
  Point3(5.5, 1.0, -50.0));
  Vector3 v2(Vector3(0.5, 0.0, 0.0));
  Bias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), bias);

  // Make sure linearization is correct. Quaternion finite differences need a
  // larger step to avoid roundoff at the strict Jacobian tolerance on Linux.
#ifdef GTSAM_USE_QUATERNIONS
  constexpr double diffDelta = 1e-7;
#else
  constexpr double diffDelta = 1e-8;
#endif
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, PredictPositionAndVelocity) {
  gttic(PredictPositionAndVelocity);
  Bias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 measuredOmega{0, 0, 0};  // M_PI/10.0+0.3;
  Vector3 measuredAcc{0, 1, -kGravity};
  double deltaT = 0.001;

  PIM pim(testing::Params(), Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  Pose3 x1;
  Vector3 v1(0, 0.0, 0.0);
  NavState actual = pim.predict(NavState(x1, v1), bias);
  NavState expected(Rot3(), Point3(0, 0.5, 0), Vector3(0, 1, 0));
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, PredictRotation) {
  gttic(PredictRotation);
  Bias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 measuredOmega{0, 0, M_PI / 10};  // M_PI/10.0+0.3;
  Vector3 measuredAcc{0, 0, -kGravity};
  double deltaT = 0.001;

  PIM pim(testing::Params(),
      Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  NavState actual = pim.predict(NavState(), bias);
  NavState expected(Rot3::Ypr(M_PI / 10, 0, 0), Point3(0,0,0), Z_3x1);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Checks a rotating, accelerating prediction against backend regressions.
TEST_PIM(ImuFactor, PredictArbitrary) {
  gttic(PredictArbitrary);
  Pose3 x1;
  const Vector3 v1(0, 0, 0);

  const AcceleratingScenario scenario(x1.rotation(), x1.translation(), v1,
                                      Vector3(0.1, 0.2, 0),
                                      Vector3(M_PI / 10, M_PI / 10, M_PI / 10));

  const double T = 3.0;  // seconds
  ScenarioRunner runner(scenario, testing::Params(), T / 10);
  //
  //  PIM pim = runner.integrate(T);
  //  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 1e-9));
  //
  //  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  //  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
  //////////////////////////////////////////////////////////////////////////////////

  Bias biasHat(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));

  // Measurements
  Vector3 measuredOmega = runner.actualAngularVelocity(0);
  Vector3 measuredAcc = runner.actualSpecificForce(0);

  auto p = testing::Params();
  p->integrationCovariance =
      Z_3x3;  // MonteCarlo does not sample integration noise
  PIM pim(p, biasHat);
  Bias bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
  //  EXPECT(MonteCarlo(pim, NavState(x1, v1), bias, 0.1, {}, measuredAcc,
  //  measuredOmega,
  //                    Vector3::Constant(accNoiseVar),
  //                    Vector3::Constant(omegaNoiseVar), 100000));

  double dt = 0.001;
  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, dt);

  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  NavState actual = pim.predict(NavState(x1, v1), bias);

  // Regression test for Imu Refactor
  Rot3 expectedR(                               //
      +0.903715275, -0.250741668, 0.347026393,  //
      +0.347026393, 0.903715275, -0.250741668,  //
      -0.250741668, 0.347026393, 0.903715275);
  Point3 expectedT(-0.516077031, 0.57842919, 0.0876478403);
  Vector3 expectedV(-1.62337767, 1.57954409, 0.343833571);
  if constexpr (std::is_same_v<PIM, PreintegratedImuMeasurementsG>) {
    // Gal3 uses the exact group exponential for each simultaneous constant
    // angular-rate/specific-force sample and a right-applied bias correction,
    // rather than the legacy Euler update and additive correction.
    expectedT = Point3(-0.516939281, 0.579119437, 0.0878198436);
    expectedV = Vector3(-1.62514265, 1.58080564, 0.344337002);
  }
  NavState expected(expectedR, expectedT, expectedV);
  EXPECT(assert_equal(expected, actual, 1e-7));
}

/* ************************************************************************* */
TEST_PIM(ImuFactor, bodyPSensorNoBias) {
  gttic(bodyPSensorNoBias);
  Bias bias(Vector3(0, 0, 0), Vector3(0, 0.1, 0)); // Biases (acc, rot)

  // Rotate sensor (z-down) to body (same as navigation) i.e. z-up
  auto p = testing::Params();
  p->n_gravity = Vector3(0, 0, -kGravity); // z-up nav frame
  p->body_P_sensor = Pose3(Rot3::Ypr(0, 0, M_PI), Point3(0, 0, 0));

  // Measurements
  // Gyroscope measurement is the angular velocity of sensor w.r.t nav frame in sensor frame
  Vector3 s_omegaMeas_ns(0, 0.1, M_PI / 10);
  // Acc measurement is acceleration of sensor in the sensor frame, when stationary,
  // table exerts an equal and opposite force w.r.t gravity
  Vector3 s_accMeas(0, 0, -kGravity);
  double dt = 0.001;

  PIM pim(p, bias);

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(s_accMeas, s_omegaMeas_ns, dt);

  // Create factor
  ImuFactorT<PIM> factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  NavState actual = pim.predict(NavState(), bias);

  Pose3 expectedPose(Rot3::Ypr(-M_PI / 10, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expectedPose, actual.pose()));

  Vector3 expectedVelocity(0, 0, 0);
  EXPECT(assert_equal(Vector(expectedVelocity), Vector(actual.velocity())));
}

/* ************************************************************************* */
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

TEST_PIM(ImuFactor, bodyPSensorWithBias) {
  gttic(bodyPSensorWithBias);
  using noiseModel::Diagonal;
  typedef Bias Bias;

  int numPoses = 10;
  Vector6 noiseBetweenBiasSigma;
  noiseBetweenBiasSigma << Vector3(2.0e-5, 2.0e-5, 2.0e-5), Vector3(3.0e-6,
      3.0e-6, 3.0e-6);
  SharedDiagonal biasNoiseModel = Diagonal::Sigmas(noiseBetweenBiasSigma);

  // Measurements
  // Sensor frame is z-down
  // Gyroscope measurement is the angular velocity of sensor w.r.t nav frame in sensor frame
  Vector3 measuredOmega(0, 0.01, 0);
  // Acc measurement is acceleration of sensor in the sensor frame, when stationary,
  // table exerts an equal and opposite force w.r.t gravity
  Vector3 measuredAcc(0, 0, -kGravity);

  auto p = testing::Params();
  p->n_gravity = Vector3(0, 0, -kGravity);
  p->body_P_sensor = Pose3(Rot3::Ypr(0, 0, M_PI), Point3(0,0,0));
  p->accelerometerCovariance = 1e-7 * I_3x3;
  p->gyroscopeCovariance = 1e-8 * I_3x3;
  p->integrationCovariance = 1e-9 * I_3x3;
  double deltaT = 0.005;

  //   Specify noise values on priors
  Vector6 priorNoisePoseSigmas(Vector{{0.001, 0.001, 0.001, 0.01, 0.01, 0.01}});
  Vector3 priorNoiseVelSigmas(Vector{{0.1, 0.1, 0.1}});
  Vector6 priorNoiseBiasSigmas(Vector{{0.1, 0.1, 0.1, 0.5e-1, 0.5e-1, 0.5e-1}});
  SharedDiagonal priorNoisePose = Diagonal::Sigmas(priorNoisePoseSigmas);
  SharedDiagonal priorNoiseVel = Diagonal::Sigmas(priorNoiseVelSigmas);
  SharedDiagonal priorNoiseBias = Diagonal::Sigmas(priorNoiseBiasSigmas);
  Vector3 zeroVel(0, 0, 0);

  // Create a factor graph with priors on initial pose, velocity and bias
  NonlinearFactorGraph graph;
  Values values;

  graph.addPrior(X(0), Pose3(), priorNoisePose);
  values.insert(X(0), Pose3());

  graph.addPrior(V(0), zeroVel, priorNoiseVel);
  values.insert(V(0), zeroVel);

  // The key to this test is that we specify the bias, in the sensor frame, as known a priori
  // We also create factors below that encode our assumption that this bias is constant over time
  // In theory, after optimization, we should recover that same bias estimate
  Bias priorBias(Vector3(0, 0, 0), Vector3(0, 0.01, 0)); // Biases (acc, rot)
  graph.addPrior(B(0), priorBias, priorNoiseBias);
  values.insert(B(0), priorBias);

  // Now add IMU factors and bias noise models
  Bias zeroBias(Vector3(0, 0, 0), Vector3(0, 0, 0));
  for (int i = 1; i < numPoses; i++) {
    PIM pim(p, priorBias);
    for (int j = 0; j < 200; ++j)
      pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

    // Create factors
    using FACTOR = ImuFactorT<PIM>;
    graph.emplace_shared<FACTOR>(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), pim);
    graph.emplace_shared<BetweenFactor<Bias> >(B(i - 1), B(i), zeroBias, biasNoiseModel);

    values.insert(X(i), Pose3());
    values.insert(V(i), zeroVel);
    values.insert(B(i), priorBias);
  }

  // Finally, optimize, and get bias at last time step
  Values result = LevenbergMarquardtOptimizer(graph, values).optimize();
  Bias biasActual = result.at<Bias>(B(numPoses - 1));

  // And compare it with expected value (our prior)
  Bias biasExpected(Vector3(0, 0, 0), Vector3(0, 0.01, 0));
  EXPECT(assert_equal(biasExpected, biasActual, 1e-3));
}

/* ************************************************************************* */
#if defined(GTSAM_TANGENT_PREINTEGRATION) && \
    !defined(GTSAM_LIEGROUP_PREINTEGRATION)
static const double kVelocity = 2.0, kAngularVelocity = M_PI / 6;

struct ImuFactorMergeTest {
  std::shared_ptr<PreintegrationParams> p_;
  const ConstantTwistScenario forward_, loop_;

  ImuFactorMergeTest()
      : p_(PreintegrationParams::MakeSharedU(kGravity)),
        forward_(kZero, Vector3(kVelocity, 0, 0)),
        loop_(Vector3(0, -kAngularVelocity, 0), Vector3(kVelocity, 0, 0)) {
    // arbitrary noise values
    p_->gyroscopeCovariance = I_3x3 * 0.01;
    p_->accelerometerCovariance = I_3x3 * 0.03;
  }

  int TestScenario(TestResult& result_, const std::string& name_,
                           const Scenario& scenario,
                           const Bias& bias01,
                           const Bias& bias12, double tol) {
    // Test merge by creating a 01, 12, and 02 PreintegratedRotation,
    // then checking the merge of 01-12 matches 02.
    PreintegratedImuMeasurements pim01(p_, bias01);
    PreintegratedImuMeasurements pim12(p_, bias12);
    PreintegratedImuMeasurements pim02_expected(p_, bias01);

    double deltaT = 0.01;
    ScenarioRunner runner(scenario, p_, deltaT);
    // TODO(frank) can this loop just go into runner ?
    for (int i = 0; i < 100; i++) {
      double t = i * deltaT;
      // integrate the measurements appropriately
      Vector3 accel_meas = runner.actualSpecificForce(t);
      Vector3 omega_meas = runner.actualAngularVelocity(t);
      pim02_expected.integrateMeasurement(accel_meas, omega_meas, deltaT);
      if (i < 50) {
        pim01.integrateMeasurement(accel_meas, omega_meas, deltaT);
      } else {
        pim12.integrateMeasurement(accel_meas, omega_meas, deltaT);
      }
    }
    auto actual_pim02 = ImuFactor::Merge(pim01, pim12);
    EXPECT(assert_equal(pim02_expected.preintegrated(),
                        actual_pim02.preintegrated(), tol));
    EXPECT(assert_equal(pim02_expected, actual_pim02, tol));

    auto factor01 =
        std::make_shared<ImuFactor>(X(0), V(0), X(1), V(1), B(0), pim01);
    auto factor12 =
        std::make_shared<ImuFactor>(X(1), V(1), X(2), V(2), B(0), pim12);
    auto factor02_expected = std::make_shared<ImuFactor>(
        X(0), V(0), X(2), V(2), B(0), pim02_expected);

    ImuFactor::shared_ptr factor02_merged = ImuFactor::Merge(factor01, factor12);
    EXPECT(assert_equal(*factor02_expected, *factor02_merged, tol));
    return result_.getFailureCount();
  }

  void TestScenarios(TestResult& result_, const std::string& name_,
                     const Bias& bias01,
                     const Bias& bias12, double tol) {
    for (auto scenario : {forward_, loop_})
      TestScenario(result_, name_, scenario, bias01, bias12, tol);
  }
};

/* ************************************************************************* */
// Test case with zero biases
TEST(ImuFactor, MergeZeroBias) {
  ImuFactorMergeTest mergeTest;
  mergeTest.TestScenarios(result_, name_, kZeroBias, kZeroBias, 1e-4);
}

// Test case with identical biases: we expect an exact answer.
TEST(ImuFactor, MergeConstantBias) {
  ImuFactorMergeTest mergeTest;
  Bias bias(Vector3(0.03, -0.02, 0.01), Vector3(-0.01, 0.02, -0.03));
  mergeTest.TestScenarios(result_, name_, bias, bias, 1e-4);
}

// Test case with different biases where we expect there to be some variation.
TEST(ImuFactor, MergeChangingBias) {
  ImuFactorMergeTest mergeTest;
  mergeTest.TestScenarios(result_, name_,
      Bias(Vector3(0.03, -0.02, 0.01), Vector3(-0.01, 0.02, -0.03)),
      Bias(Vector3(0.01, 0.02, 0.03), Vector3(0.03, -0.02, 0.01)), 1e-1);
}

// Test case with non-zero coriolis
TEST(ImuFactor, MergeWithCoriolis) {
  ImuFactorMergeTest mergeTest;
  mergeTest.p_->omegaCoriolis = Vector3(0.1, 0.2, -0.1);
  mergeTest.TestScenarios(result_, name_, kZeroBias, kZeroBias, 1e-4);
}
#endif

/* ************************************************************************* */
// Same values as pre-integration test but now testing covariance
TEST_PIM(ImuFactor, CheckCovariance) {
  gttic(CheckCovariance);
  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;

  PIM actual(testing::Params());
  actual.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  Matrix9 expected{{1.0577e-08, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 1.0577e-08, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 1.0577e-08, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 5.00868e-05, 0, 0, 3.47222e-07, 0, 0},
                   {0, 0, 0, 0, 5.00868e-05, 0, 0, 3.47222e-07, 0},
                   {0, 0, 0, 0, 0, 5.00868e-05, 0, 0, 3.47222e-07},
                   {0, 0, 0, 3.47222e-07, 0, 0, 1.38889e-06, 0, 0},
                   {0, 0, 0, 0, 3.47222e-07, 0, 0, 1.38889e-06, 0},
                   {0, 0, 0, 0, 0, 3.47222e-07, 0, 0, 1.38889e-06}};
  EXPECT(assert_equal(expected, actual.preintMeasCov()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  auto result = TestRegistry::runAllTests(tr);
#ifdef ENABLE_TIMING
  tictoc_print_();
#endif
  return result;
}
/* ************************************************************************* */
