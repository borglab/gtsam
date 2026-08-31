/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved.
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLieGroupPreintegration.cpp
 * @brief Tests for NavState SE_2(3) Lie-group IMU preintegration.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/LieGroupPreintegration.h>

#include "imuFactorTesting.h"

/* ************************************************************************* */
namespace integration_fixture {

constexpr double kDt = 0.1;
const Vector3 kAcceleration{0.1, 0.2, 9.8};
const Vector3 kOmega{0.1, -0.2, 0.3};

class TestableLieGroupPreintegration : public LieGroupPreintegration {
 public:
  using LieGroupPreintegration::LieGroupPreintegration;
  using LieGroupPreintegration::updateFactor;

  void setDeltaXij(const NavState& state) { deltaXij_ = state; }
};

NavState integrate(const NavState& initial, const Vector3& acceleration,
                   const Vector3& omega) {
  TestableLieGroupPreintegration pim(testing::Params());
  pim.setDeltaXij(initial);
  pim.updateFactor(acceleration, omega, kDt);
  return pim.deltaXij();
}

// Verifies one IMU increment is applied with NavState's group exponential map.
TEST(LieGroupPreintegration, UsesNavStateExpmap) {
  const NavState physicalIncrement(Rot3::Expmap(kDt * kOmega),
                                   0.5 * kDt * kDt * kAcceleration,
                                   kDt * kAcceleration);
  const Vector9 groupTangent = NavState::Logmap(physicalIncrement);
  EXPECT(assert_equal(NavState().expmap(groupTangent),
                      integrate(NavState(), kAcceleration, kOmega), 1e-12));
}

// Verifies state, accelerometer, and gyroscope propagation Jacobians.
TEST(LieGroupPreintegration, UpdateJacobians) {
  const NavState initial(Rot3::Ypr(0.3, -0.2, 0.1), Point3(1.0, -2.0, 0.5),
                         Vector3(0.4, 0.2, -0.1));
  TestableLieGroupPreintegration pim(testing::Params());
  pim.setDeltaXij(initial);
  Matrix9 F;
  Matrix93 G1, G2;
  pim.updateFactor(kAcceleration, kOmega, kDt, F, G1, G2);

  auto propagate = [](const NavState& state, const Vector3& acceleration,
                      const Vector3& omega) {
    return integrate(state, acceleration, omega);
  };

  EXPECT(assert_equal(
      numericalDerivative31(propagate, initial, kAcceleration, kOmega), F,
      1e-8));
  EXPECT(assert_equal(
      numericalDerivative32(propagate, initial, kAcceleration, kOmega), G1,
      1e-8));
  EXPECT(assert_equal(
      numericalDerivative33(propagate, initial, kAcceleration, kOmega), G2,
      1e-8));
}

}  // namespace integration_fixture
/* ************************************************************************* */
namespace bias_fixture {

Matrix96 rightBiasJacobian(const LieGroupPreintegration& pim) {
  const Matrix3 deltaRotationTranspose = pim.deltaRij().transpose();
  Matrix96 result;
  result << Z_3x3, pim.delRdelBiasOmega(),
      deltaRotationTranspose * pim.delPdelBiasAcc(),
      deltaRotationTranspose * pim.delPdelBiasOmega(),
      deltaRotationTranspose * pim.delVdelBiasAcc(),
      deltaRotationTranspose * pim.delVdelBiasOmega();
  return result;
}

// Verifies accumulated component Jacobians with respect to the bias hat.
TEST(LieGroupPreintegration, AccumulatedBiasJacobians) {
  const testing::SomeMeasurements measurements;

  auto deltaRotation = [&](const Vector3& accelerationBias,
                           const Vector3& omegaBias) {
    LieGroupPreintegration pim(testing::Params(),
                               Bias(accelerationBias, omegaBias));
    testing::integrateMeasurements(measurements, &pim);
    return pim.deltaRij();
  };
  auto deltaPosition = [&](const Vector3& accelerationBias,
                           const Vector3& omegaBias) {
    LieGroupPreintegration pim(testing::Params(),
                               Bias(accelerationBias, omegaBias));
    testing::integrateMeasurements(measurements, &pim);
    return pim.deltaPij();
  };
  auto deltaVelocity = [&](const Vector3& accelerationBias,
                           const Vector3& omegaBias) {
    LieGroupPreintegration pim(testing::Params(),
                               Bias(accelerationBias, omegaBias));
    testing::integrateMeasurements(measurements, &pim);
    return pim.deltaVij();
  };

  LieGroupPreintegration pim(testing::Params());
  testing::integrateMeasurements(measurements, &pim);

  EXPECT(assert_equal(numericalDerivative21(deltaRotation, kZero, kZero),
                      Matrix3(Z_3x3)));
  EXPECT(assert_equal(numericalDerivative22(deltaRotation, kZero, kZero),
                      pim.delRdelBiasOmega(), 1e-3));
  EXPECT(assert_equal(numericalDerivative21(deltaPosition, kZero, kZero),
                      pim.delPdelBiasAcc(), 1e-6));
  EXPECT(assert_equal(numericalDerivative22(deltaPosition, kZero, kZero),
                      pim.delPdelBiasOmega(), 1e-3));
  EXPECT(assert_equal(numericalDerivative21(deltaVelocity, kZero, kZero),
                      pim.delVdelBiasAcc(), 1e-6));
  EXPECT(assert_equal(numericalDerivative22(deltaVelocity, kZero, kZero),
                      pim.delVdelBiasOmega(), 1e-3));
}

// Verifies the right-local bias Jacobian implements Brossard's recursion.
TEST(LieGroupPreintegration, RightBiasJacobianMatchesReintegration) {
  const testing::SomeMeasurements measurements;
  LieGroupPreintegration nominal(testing::Params());
  testing::integrateMeasurements(measurements, &nominal);

  auto reintegratedCorrection = [&](const Bias& bias) {
    LieGroupPreintegration reintegrated(testing::Params(), bias);
    testing::integrateMeasurements(measurements, &reintegrated);
    return nominal.deltaXij().logmap(reintegrated.deltaXij());
  };

  const Matrix96 numerical =
      numericalDerivative11<Vector9, Bias>(reintegratedCorrection, Bias());
  EXPECT(assert_equal(numerical, rightBiasJacobian(nominal), 1e-6));
}

// Verifies the nonlinear Lie-group bias correction and its Jacobian.
TEST(LieGroupPreintegration, BiasCorrectedDeltaJacobian) {
  LieGroupPreintegration pim(testing::Params());
  testing::integrateMeasurements(testing::SomeMeasurements(), &pim);
  const Bias bias(Vector3{1e-3, -2e-3, 3e-3}, Vector3{-4e-4, 5e-4, -6e-4});

  Matrix96 actualJacobian;
  pim.biasCorrectedDelta(bias, actualJacobian);
  auto corrected = [&](const Bias& value) {
    return pim.biasCorrectedDelta(value);
  };
  EXPECT(assert_equal(numericalDerivative11(corrected, bias), actualJacobian,
                      1e-6));
}

// Verifies bias correction applies one complete SE_2(3) update on the right.
TEST(LieGroupPreintegration, RightAppliedBiasCorrection) {
  LieGroupPreintegration pim(testing::Params());
  testing::integrateMeasurements(testing::SomeMeasurements(), &pim);
  const Bias bias(Vector3{0.02, -0.01, 0.03}, Vector3{-0.015, 0.01, -0.02});
  const Bias biasIncrement = bias - pim.biasHat();

  const Vector9 correctionTangent =
      rightBiasJacobian(pim) * biasIncrement.vector();
  const NavState corrected =
      pim.deltaXij().compose(NavState::Expmap(correctionTangent));
  const Vector9 expected = internal::navStateComponentWiseLocalCoordinates(
      NavState(), corrected);
  EXPECT(assert_equal(expected, pim.biasCorrectedDelta(bias), 1e-12));

  // The former implementation added the correction's translation columns
  // without rotating them through the preintegrated state.
  const NavState correction = NavState::Expmap(correctionTangent);
  Vector9 additive;
  NavState::dR(additive) = Rot3::Logmap(corrected.attitude());
  NavState::dP(additive) = pim.deltaPij() + correction.position();
  NavState::dV(additive) = pim.deltaVij() + correction.velocity();
  EXPECT((expected.tail<6>() - additive.tail<6>()).norm() > 1e-6);
}

}  // namespace bias_fixture
/* ************************************************************************* */
namespace factor_fixture {

const NavState kState1(Rot3::Ypr(0.4, -0.2, 0.1), Point3(1.0, -2.0, 0.5),
                       Vector3(0.4, 0.2, -0.1));
const NavState kState2(Rot3::Ypr(-0.2, 0.3, -0.1),
                       Point3(-0.5, 1.2, 2.0),
                       Vector3(-0.3, 0.6, 0.2));

LieGroupPreintegration makePim() {
  LieGroupPreintegration pim(testing::Params());
  testing::integrateMeasurements(testing::SomeMeasurements(), &pim);
  return pim;
}

// Verifies Legacy mode uses the SE_2(3) logarithm at finite residuals.
TEST(LieGroupPreintegration, ComputeErrorUsesGroupLogmap) {
  const LieGroupPreintegration pim = makePim();
  const Bias bias;
  const NavState predicted = pim.predict(kState1, bias);
  const Vector9 expected = kState2.logmap(predicted);
  const Vector9 actual =
      internal::preintegrationError(pim, kState1, kState2, bias);
  const Vector9 componentWise =
      internal::navStateComponentWiseLocalCoordinates(kState2, predicted);

  EXPECT(assert_equal(expected, actual, 1e-12));
  EXPECT((actual - componentWise).norm() > 1e-3);
}

// Verifies PreintegrationBase error Jacobians with this backend.
TEST(LieGroupPreintegration, ComputeErrorJacobians) {
  const LieGroupPreintegration pim = makePim();
  const Bias bias;
  Matrix9 actualH1, actualH2;
  Matrix96 actualH3;
  internal::preintegrationError(pim, kState1, kState2, bias, actualH1,
                                actualH2,
                                actualH3);
  auto error = [&](const NavState& a, const NavState& b, const Bias& c) {
    return internal::preintegrationError(pim, a, b, c);
  };

  EXPECT(assert_equal(numericalDerivative31(error, kState1, kState2, bias),
                      actualH1, 1e-8));
  EXPECT(assert_equal(numericalDerivative32(error, kState1, kState2, bias),
                      actualH2, 1e-8));
  EXPECT(assert_equal(numericalDerivative33(error, kState1, kState2, bias),
                      actualH3, 1e-8));
}

}  // namespace factor_fixture
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
