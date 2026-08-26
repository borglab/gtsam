/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRelativeTranslationFactor.cpp
 * @brief   Unit tests for the SE-Sync translation term.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/RelativeTranslationFactor.h>

#include <cmath>

using namespace gtsam;

/* ************************************************************************* */
namespace Rot3Fixture {

const Key kR = Symbol('r', 0);
const Key kT1 = Symbol('t', 0);
const Key kT2 = Symbol('t', 1);

// evaluateError reproduces sqrt(weight) * (tj - ti - Ri * measured).
TEST(RelativeTranslationFactor, EvaluateErrorMatchesDirectComputation) {
  const Rot3 Ri = Rot3::RzRyRx(0.1, -0.2, 0.3);
  const Vector3 ti(0.5, -0.3, 0.2), tj(1.5, 0.4, -0.1);
  const Vector3 measured(0.2, 0.1, 0.05);
  const double weight = 2.5;

  const RelativeTranslationFactor3 factor(kR, kT1, kT2, measured, weight);
  const Vector actual = factor.evaluateError(Ri, ti, tj, {}, {}, {});
  const Vector expected =
      std::sqrt(weight) * (tj - ti - Ri.rotate(measured));
  EXPECT(assert_equal(expected, actual, 1e-12));
}

// Analytic Jacobians match numerical derivatives at a generic point.
TEST(RelativeTranslationFactor, JacobiansMatchNumericalDerivative) {
  const Rot3 Ri = Rot3::RzRyRx(0.2, 0.1, -0.3);
  const Vector3 ti(0.1, 0.2, 0.3), tj(-0.4, 0.1, 0.6), measured(0.3, -0.2, 0.1);
  const double weight = 1.7;
  const RelativeTranslationFactor3 factor(kR, kT1, kT2, measured, weight);

  Matrix H1, H2, H3;
  factor.evaluateError(Ri, ti, tj, H1, H2, H3);

  auto f = [&](const Rot3& R, const Vector3& a, const Vector3& b) {
    return factor.evaluateError(R, a, b, {}, {}, {});
  };
  const Matrix numH1 = numericalDerivative31<Vector, Rot3, Vector3, Vector3>(
      f, Ri, ti, tj);
  const Matrix numH2 = numericalDerivative32<Vector, Rot3, Vector3, Vector3>(
      f, Ri, ti, tj);
  const Matrix numH3 = numericalDerivative33<Vector, Rot3, Vector3, Vector3>(
      f, Ri, ti, tj);

  EXPECT(assert_equal(numH1, H1, 1e-6));
  EXPECT(assert_equal(numH2, H2, 1e-6));
  EXPECT(assert_equal(numH3, H3, 1e-6));
}

// The whole linearize() path, which is what the optimizer consumes: it adds the
// noise model's whitening on top of the raw Jacobians checked above.
TEST(RelativeTranslationFactor, LinearizationMatchesNumericalJacobians) {
  const RelativeTranslationFactor3 factor(kR, kT1, kT2, Vector3(0.1, 0.2, -0.1),
                                          0.8);
  Values values;
  values.insert(kR, Rot3::RzRyRx(-0.1, 0.2, 0.15));
  values.insert(kT1, Vector3(0.3, -0.1, 0.4));
  values.insert(kT2, Vector3(-0.2, 0.5, 0.1));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-6, 1e-6);
}

// The QCQP cost at rank d equals 0.5 * ||evaluateError||^2 evaluated on the
// exact (unlifted) rank-d representation: rotation is R', translations are
// row vectors t'.
TEST(RelativeTranslationFactor, QcqpFactorsMatchesNonlinearErrorAtRankD) {
  const Rot3 Ri = Rot3::RzRyRx(-0.1, 0.2, 0.15);
  const Vector3 ti(0.3, -0.1, 0.4), tj(-0.2, 0.5, 0.1);
  const Vector3 measured(0.1, 0.2, -0.1);
  const double weight = 0.8;
  const RelativeTranslationFactor3 factor(kR, kT1, kT2, measured, weight);

  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  factor.qcqpFactors(&costs, &constraints, /*columnDimension=*/3);
  EXPECT(constraints.empty());
  EXPECT_LONGS_EQUAL(1, static_cast<long>(costs.size()));

  Values Y;
  Y.insert(kR, Matrix(Ri.matrix().transpose()));
  Y.insert(kT1, Matrix(ti.transpose()));
  Y.insert(kT2, Matrix(tj.transpose()));

  const double qcqpCost = costs.error(Y);
  const double directCost =
      0.5 * factor.evaluateError(Ri, ti, tj, {}, {}, {}).squaredNorm();
  EXPECT_DOUBLES_EQUAL(directCost, qcqpCost, 1e-10);
}

// Non-positive weight is rejected at construction.
TEST(RelativeTranslationFactor, RejectsNonPositiveWeight) {
  CHECK_EXCEPTION(
      RelativeTranslationFactor3(kR, kT1, kT2, Vector3::Zero(), 0.0),
      std::invalid_argument);
  CHECK_EXCEPTION(
      RelativeTranslationFactor3(kR, kT1, kT2, Vector3::Zero(), -1.0),
      std::invalid_argument);
}

// qcqpFactors rejects a staircase rank narrower than the ambient dimension.
TEST(RelativeTranslationFactor, QcqpFactorsRejectsSmallColumnDimension) {
  const RelativeTranslationFactor3 factor(kR, kT1, kT2, Vector3(0.1, 0.2, 0.3),
                                          1.0);
  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  CHECK_EXCEPTION(
      factor.qcqpFactors(&costs, &constraints, /*columnDimension=*/2),
      std::invalid_argument);
}

}  // namespace Rot3Fixture
/* ************************************************************************* */
namespace Rot2Fixture {

const Key kR = Symbol('r', 0);
const Key kT1 = Symbol('t', 0);
const Key kT2 = Symbol('t', 1);

// Same cross-check as the Rot3 fixture, specialized to d=2.
TEST(RelativeTranslationFactor, QcqpFactorsMatchesNonlinearErrorAtRankD2) {
  const Rot2 Ri = Rot2::fromAngle(0.4);
  const Vector2 ti(0.2, -0.3), tj(0.5, 0.1), measured(0.1, -0.2);
  const double weight = 1.3;
  const RelativeTranslationFactor2 factor(kR, kT1, kT2, measured, weight);

  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  factor.qcqpFactors(&costs, &constraints, /*columnDimension=*/2);

  Values Y;
  Y.insert(kR, Matrix(Ri.matrix().transpose()));
  Y.insert(kT1, Matrix(ti.transpose()));
  Y.insert(kT2, Matrix(tj.transpose()));

  const double qcqpCost = costs.error(Y);
  const double directCost =
      0.5 * factor.evaluateError(Ri, ti, tj, {}, {}, {}).squaredNorm();
  EXPECT_DOUBLES_EQUAL(directCost, qcqpCost, 1e-10);
}

}  // namespace Rot2Fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
