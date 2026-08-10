/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCombinedImuFactorWithGravity.cpp
 * @brief   Unit test for the gravity-aware CombinedImuFactor variants
 * @author  Nikhil Khedekar
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactorWithGravity.h>
#include <gtsam/navigation/ImuFactor.h>  // for TEST_PIM's plain PIM
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "imuFactorTesting.h"

/* ************************************************************************* */
namespace combined {
// Create default parameters with Z-down, as in testCombinedImuFactor.cpp
static std::shared_ptr<PreintegratedCombinedMeasurements::Params> Params(
    const Matrix3& biasAccCovariance = Matrix3::Zero(),
    const Matrix3& biasOmegaCovariance = Matrix3::Zero()) {
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  p->biasAccCovariance = biasAccCovariance;
  p->biasOmegaCovariance = biasOmegaCovariance;
  return p;
}
}  // namespace combined
/* ************************************************************************* */

/* ************************************************************************* */
TEST_PIM(CombinedImuFactorWithGravity, Jacobians) {
  using symbol_shorthand::G;
  Bias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3));
  Bias bias2(Vector3(0.2, 0.2, 0), Vector3(1, 0, 0.3));
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)), Point3(5.0, 1.0, -50.0));
  Vector3 v1(0.5, 0.0, 0.0);
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(0.5, 0.0, 0.0);

  auto p = combined::Params();
  CombinedPIM pim(p, Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));
  const Vector3 measuredAcc =
      x1.rotation().unrotate(-p->n_gravity) + Vector3(0.2, 0.0, 0.0);
  const Vector3 measuredOmega(0, 0, M_PI / 10.0 + 0.3);
  pim.integrateMeasurement(measuredAcc, measuredOmega, 1.0);

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), bias);
  values.insert(B(2), bias2);

  {
    CombinedImuFactorWithGravityT<CombinedPIM, Unit3> factor(
        X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim);
    Values v = values;
    v.insert(G(0), Unit3(0.1, -0.2, -1.0));
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, v, 1e-7, 1e-3);
  }
  {
    CombinedImuFactorWithGravityT<CombinedPIM, Point3> factor(
        X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim);
    Values v = values;
    v.insert(G(0), Point3(0.4, -0.6, -9.5));
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, v, 1e-7, 1e-3);
  }
}

/* ************************************************************************* */
TEST(CombinedImuFactorWithGravity, Equals) {
  using symbol_shorthand::G;
  auto p = combined::Params();
  PreintegratedCombinedMeasurements pim(p);

  CombinedImuFactorWithGravityDirection d1(X(1), V(1), X(2), V(2), B(1), B(2),
                                           G(0), pim);
  CombinedImuFactorWithGravityDirection d2(X(1), V(1), X(2), V(2), B(1), B(2),
                                           G(0), pim);
  CombinedImuFactorWithGravityDirection d3(X(1), V(1), X(2), V(2), B(1), B(2),
                                           G(0), pim, 1.62);
  EXPECT(d1.equals(d2));
  EXPECT(!d1.equals(d3));

  // The Point3 parametrization does not use the magnitude, so factors that
  // differ only in the (inert) stored magnitude must compare equal:
  CombinedImuFactorWithGravityVector v1(X(1), V(1), X(2), V(2), B(1), B(2),
                                        G(0), pim);
  CombinedImuFactorWithGravityVector v2(X(1), V(1), X(2), V(2), B(1), B(2),
                                        G(0), pim);
  EXPECT(v1.equals(v2));

  // Comparing against a different factor type must not crash:
  EXPECT(!d1.equals(v1));
  EXPECT(!v1.equals(d1));
}

/* ************************************************************************* */
TEST(CombinedImuFactorWithGravity, ConstructorValidation) {
  using symbol_shorthand::G;
  auto p = combined::Params();
  PreintegratedCombinedMeasurements pim(p);
  // The Unit3 parametrization requires a positive magnitude:
  CHECK_EXCEPTION(CombinedImuFactorWithGravityDirection(
                      X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim, 0.0),
                  std::invalid_argument);
  // The Point3 parametrization optimizes the magnitude as part of the gravity
  // variable, so providing one is rejected (use VectorNormFactor<3> instead):
  CHECK_EXCEPTION(CombinedImuFactorWithGravityVector(
                      X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim, 9.81),
                  std::invalid_argument);
}

/* ************************************************************************* */
TEST_PIM(CombinedImuFactorWithGravity, ConsistentWithCombinedImuFactor) {
  using symbol_shorthand::G;
  auto p = combined::Params();
  CombinedPIM pim(p);
  pim.integrateMeasurement(Vector3(0.1, 0.2, -9.81), Vector3(0.1, 0, 0), 0.5);

  const Pose3 x1;
  const Vector3 v1(0.1, 0, 0);
  const Pose3 x2;
  const Vector3 v2(0.1, 0, 0);
  const Bias bias1, bias2;

  CombinedImuFactorT<CombinedPIM> plain(X(1), V(1), X(2), V(2), B(1), B(2), pim);
  CombinedImuFactorWithGravityT<CombinedPIM, Unit3> withGravity(
      X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim);
  // With the params' gravity direction, the errors must agree:
  EXPECT(assert_equal(
      plain.evaluateError(x1, v1, x2, v2, bias1, bias2),
      withGravity.evaluateError(x1, v1, x2, v2, bias1, bias2,
                                Unit3(p->n_gravity))));
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
