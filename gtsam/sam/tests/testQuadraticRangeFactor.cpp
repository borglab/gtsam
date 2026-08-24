/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testQuadraticRangeFactor.cpp
 * @brief   Unit tests for the CORA range term.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sam/QuadraticRangeFactor.h>

#include <cmath>

using namespace gtsam;

/* ************************************************************************* */
namespace Range3Fixture {

const Key kTi = Symbol('t', 0);
const Key kTarget = Symbol('l', 0);
const Key kU = Symbol('u', 0);

// evaluateError reproduces sqrt(weight) * (target - ti - range * u).
TEST(QuadraticRangeFactor, EvaluateErrorMatchesDirectComputation) {
  const Vector3 ti(0.5, -0.3, 0.2), target(2.5, 0.4, -0.1);
  const Unit3 u(Point3(0.3, -0.6, 0.7));
  const double range = 1.8, weight = 2.5;

  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, range, weight);
  const Vector actual = factor.evaluateError(ti, target, u, {}, {}, {});
  const Vector expected =
      std::sqrt(weight) * (target - ti - range * u.unitVector());
  EXPECT(assert_equal(expected, actual, 1e-12));
}

// Analytic Jacobians match numerical derivatives at a generic point. H3 is the
// one worth pinning: it maps the two-dimensional tangent of the sphere into the
// ambient residual, so a wrong basis is invisible in the error itself.
TEST(QuadraticRangeFactor, JacobiansMatchNumericalDerivative) {
  const Vector3 ti(0.1, 0.2, 0.3), target(-0.4, 0.1, 0.6);
  const Unit3 u(Point3(-0.2, 0.5, 0.4));
  const double range = 1.3, weight = 1.7;
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, range, weight);

  Matrix H1, H2, H3;
  factor.evaluateError(ti, target, u, H1, H2, H3);

  auto f = [&](const Vector3& a, const Vector3& b, const Unit3& direction) {
    return factor.evaluateError(a, b, direction, {}, {}, {});
  };
  const Matrix numH1 =
      numericalDerivative31<Vector, Vector3, Vector3, Unit3>(f, ti, target, u);
  const Matrix numH2 =
      numericalDerivative32<Vector, Vector3, Vector3, Unit3>(f, ti, target, u);
  const Matrix numH3 =
      numericalDerivative33<Vector, Vector3, Vector3, Unit3>(f, ti, target, u);

  EXPECT(assert_equal(numH1, H1, 1e-7));
  EXPECT(assert_equal(numH2, H2, 1e-7));
  EXPECT(assert_equal(numH3, H3, 1e-7));
}

// An independent check on H3 that uses no difference scheme at all: the
// first-order model must have O(h^2) error along the sphere, so halving the
// step quarters the remainder. Confirms the derivative against its definition.
TEST(QuadraticRangeFactor, JacobianSatisfiesFirstOrderTaylorModel) {
  const Vector3 ti(0.1, 0.2, 0.3), target(-0.4, 0.1, 0.6);
  const Unit3 u(Point3(0.3, 0.1, 0.95));
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, 1.3, 1.7);

  Matrix H1, H2, H3;
  const Vector residual = factor.evaluateError(ti, target, u, H1, H2, H3);

  const Vector2 delta(0.7, -0.4);
  double previous = 0.0;
  for (double step : {1e-2, 5e-3, 2.5e-3}) {
    const Unit3 moved = u.retract(step * delta);
    const double remainder =
        (factor.evaluateError(ti, target, moved, {}, {}, {}) - residual -
         H3 * (step * delta))
            .norm();
    if (previous > 0.0) EXPECT(remainder < 0.3 * previous);
    previous = remainder;
  }
}

// The whole linearize() path, which is what the optimizer consumes: it adds the
// noise model's whitening on top of the raw Jacobians checked above.
TEST(QuadraticRangeFactor, LinearizationMatchesNumericalJacobians) {
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, 1.3, 1.7);
  Values values;
  values.insert(kTi, Vector3(0.1, 0.2, 0.3));
  values.insert(kTarget, Vector3(-0.4, 0.1, 0.6));
  values.insert(kU, Unit3(Point3(0.3, 0.1, 0.95)));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-6, 1e-6);
}

// The auxiliary is a reformulation rather than an approximation: minimizing
// over the unit sphere recovers the raw range residual, attained at the
// normalized offset. This justifies the extra variable.
TEST(QuadraticRangeFactor, MinimumOverSphereRecoversRangeResidual) {
  const Vector3 ti(0.2, -0.1, 0.4), target(3.2, 1.4, -0.6);
  const double range = 2.0, weight = 3.0;
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, range, weight);

  const Vector3 offset = target - ti;
  const Unit3 best{Point3(offset)};
  const double achieved =
      factor.evaluateError(ti, target, best, {}, {}, {}).norm();
  EXPECT_DOUBLES_EQUAL(std::sqrt(weight) * std::abs(offset.norm() - range),
                       achieved, 1e-12);

  // Any other direction is worse, so the minimum is not attained off the
  // normalized offset.
  const Unit3 other(Point3(offset.z(), offset.x(), -offset.y()));
  EXPECT(factor.evaluateError(ti, target, other, {}, {}, {}).norm() > achieved);
}

// A zero-length offset leaves the direction unconstrained, so the residual is
// the range itself whichever unit vector is supplied.
TEST(QuadraticRangeFactor, CoincidentPositionsLeaveDirectionFree) {
  const Vector3 ti(1.0, 2.0, 3.0);
  const double range = 1.5, weight = 1.0;
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, range, weight);

  const double first =
      factor.evaluateError(ti, ti, Unit3(Point3(1, 0, 0)), {}, {}, {}).norm();
  const double second =
      factor.evaluateError(ti, ti, Unit3(Point3(0, 0, 1)), {}, {}, {}).norm();
  EXPECT_DOUBLES_EQUAL(range, first, 1e-12);
  EXPECT_DOUBLES_EQUAL(first, second, 1e-12);
}

// A non-positive weight or a negative range is rejected at construction.
TEST(QuadraticRangeFactor, RejectsInvalidMeasurements) {
  CHECK_EXCEPTION(QuadraticRangeFactor3(kTi, kTarget, kU, 1.0, 0.0),
                  std::invalid_argument);
  CHECK_EXCEPTION(QuadraticRangeFactor3(kTi, kTarget, kU, -1.0, 1.0),
                  std::invalid_argument);
}

// H3 must be expressed in the basis the direction type itself retracts along,
// so it has to stay correct as Unit3 switches its tangent basis. Unit3 picks
// the axis with the smallest component of u, so cover all three choices; a
// hand-rolled basis that happens to agree on one of them fails on the others.
TEST(QuadraticRangeFactor, JacobianTracksUnit3TangentBasisChoice) {
  const Vector3 ti(0.1, 0.2, 0.3), target(-0.4, 0.1, 0.6);
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, 1.3, 1.7);
  auto f = [&](const Vector3& a, const Vector3& b, const Unit3& direction) {
    return factor.evaluateError(a, b, direction, {}, {}, {});
  };

  // The three cases below have their smallest component in x, y, and z.
  for (const Point3& p : {Point3(0.1, 0.6, 0.8), Point3(0.3, 0.1, 0.95),
                          Point3(0.7, 0.7, 0.05)}) {
    const Unit3 u(p);
    Matrix H1, H2, H3;
    factor.evaluateError(ti, target, u, H1, H2, H3);
    EXPECT(assert_equal(
        Matrix(numericalDerivative33<Vector, Vector3, Vector3, Unit3>(
            f, ti, target, u)),
        H3, 1e-7));
  }
}

}  // namespace Range3Fixture
/* ************************************************************************* */
namespace Range2Fixture {

const Key kTi = Symbol('t', 0);
const Key kTarget = Symbol('l', 0);
const Key kU = Symbol('u', 0);

// The planar factor behaves the same way over Rot2, including the H3 tangent,
// which is one-dimensional there rather than two.
TEST(QuadraticRangeFactor, PlanarJacobiansMatchNumericalDerivative) {
  const Vector2 ti(0.3, -0.2), target(1.4, 0.9);
  const Rot2 u = Rot2(0.7);
  const double range = 1.1, weight = 2.0;
  const QuadraticRangeFactor2 factor(kTi, kTarget, kU, range, weight);

  Matrix H1, H2, H3;
  factor.evaluateError(ti, target, u, H1, H2, H3);

  auto f = [&](const Vector2& a, const Vector2& b, const Rot2& direction) {
    return factor.evaluateError(a, b, direction, {}, {}, {});
  };
  const Matrix numH1 =
      numericalDerivative31<Vector, Vector2, Vector2, Rot2>(f, ti, target, u);
  const Matrix numH2 =
      numericalDerivative32<Vector, Vector2, Vector2, Rot2>(f, ti, target, u);
  const Matrix numH3 =
      numericalDerivative33<Vector, Vector2, Vector2, Rot2>(f, ti, target, u);

  EXPECT(assert_equal(numH1, H1, 1e-7));
  EXPECT(assert_equal(numH2, H2, 1e-7));
  EXPECT(assert_equal(numH3, H3, 1e-7));
}

}  // namespace Range2Fixture
/* ************************************************************************* */
namespace QcqpFixture {

const Key kTi = Symbol('t', 0);
const Key kTarget = Symbol('l', 0);
const Key kU = Symbol('u', 0);

// The lifted cost evaluated at the rank-d lift agrees with the typed residual,
// which pins the whole cost block rather than one entry of it.
TEST(QuadraticRangeFactor, QcqpCostMatchesTypedResidual) {
  const Vector2 ti(0.4, -0.2), target(1.9, 0.7);
  const Rot2 u = Rot2(0.35);
  const double range = 1.4, weight = 2.25;
  const QuadraticRangeFactor2 factor(kTi, kTarget, kU, range, weight);

  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  factor.qcqpFactors(&costs, &constraints, /*columnDimension=*/2);
  EXPECT_LONGS_EQUAL(1, static_cast<long>(costs.size()));

  Values Y;
  Y.insert(kTi, Matrix(ti.transpose()));
  Y.insert(kTarget, Matrix(target.transpose()));
  Y.insert(kU, traits<Rot2>::QcqpValue<2>(u));

  const double directCost =
      0.5 * factor.evaluateError(ti, target, u, {}, {}, {}).squaredNorm();
  EXPECT_DOUBLES_EQUAL(directCost, costs.error(Y), 1e-10);
}

// The factor owns the auxiliary, so it emits that direction type's own QCQP
// constraints rather than writing them out here: three row-orthonormality
// constraints for the planar Rot2 frame.
TEST(QuadraticRangeFactor, EmitsItsAuxiliarysOwnConstraints) {
  const QuadraticRangeFactor2 factor(kTi, kTarget, kU, 1.0, 1.0);

  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  factor.qcqpFactors(&costs, &constraints, 2);

  const auto expected = traits<Rot2>::QcqpConstraints<2>();
  EXPECT_LONGS_EQUAL(static_cast<long>(expected.size()),
                     static_cast<long>(constraints.size()));
  for (size_t i = 0; i < expected.size(); ++i) {
    const auto quadratic =
        std::dynamic_pointer_cast<QuadraticEqualityConstraintFactor>(
            constraints.at(i));
    CHECK(quadratic);
    const QuadraticConstraint& constraint = quadratic->quadraticConstraint();
    EXPECT(constraint.key() == kU);
    EXPECT_DOUBLES_EQUAL(expected[i].second, constraint.b(), 1e-12);
    EXPECT(assert_equal(expected[i].first, constraint.A(), 1e-12));
  }
}

// The QCQP cost multiplies the auxiliary's leading lifted row while the typed
// residual uses u = R * e_1, so the two must be the same vector.
// traits<Rot2>::QcqpValue stores R transposed, which is what makes that hold.
TEST(QuadraticRangeFactor, LiftedLeadingRowIsTheUnitVector) {
  const Rot2 u = Rot2(0.7);
  const Matrix X = traits<Rot2>::QcqpValue<4>(u);
  EXPECT_LONGS_EQUAL(QuadraticRangeFactor2::kDirectionRows,
                     static_cast<long>(X.rows()));
  EXPECT(assert_equal(Vector2(u.c(), u.s()),
                      Vector2(X.row(0).head<2>().transpose()), 1e-12));
  // And recovery inverts it, reading only that row.
  EXPECT_DOUBLES_EQUAL(u.theta(), Rot2::atan2(X(0, 1), X(0, 0)).theta(), 1e-12);
}

// The same cost agreement in 3D, where the Unit3 auxiliary occupies one lifted
// row rather than two.
TEST(QuadraticRangeFactor, QcqpCostMatchesTypedResidualIn3D) {
  const Vector3 ti(0.4, -0.2, 0.1), target(1.9, 0.7, -0.3);
  const Unit3 u(Point3(0.3, -0.5, 0.8));
  const double range = 1.4, weight = 2.25;
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, range, weight);

  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  factor.qcqpFactors(&costs, &constraints, /*columnDimension=*/3);
  EXPECT_LONGS_EQUAL(1, static_cast<long>(costs.size()));

  Values Y;
  Y.insert(kTi, Matrix(ti.transpose()));
  Y.insert(kTarget, Matrix(target.transpose()));
  Y.insert(kU, traits<Unit3>::QcqpValue<3>(u));

  const double directCost =
      0.5 * factor.evaluateError(ti, target, u, {}, {}, {}).squaredNorm();
  EXPECT_DOUBLES_EQUAL(directCost, costs.error(Y), 1e-10);
}

// Lifting pads the variables with zero columns, so the cost at rank p > d has
// to stay equal to the cost at rank d as the staircase climbs.
TEST(QuadraticRangeFactor, QcqpCostIsUnchangedByLifting) {
  const Vector2 ti(0.4, -0.2), target(1.9, 0.7);
  const Rot2 u = Rot2(0.35);
  const QuadraticRangeFactor2 factor(kTi, kTarget, kU, 1.4, 2.25);
  const double expected =
      0.5 * factor.evaluateError(ti, target, u, {}, {}, {}).squaredNorm();

  for (int p : {2, 3, 5}) {
    NonlinearFactorGraph costs;
    NonlinearEqualityConstraints constraints;
    factor.qcqpFactors(&costs, &constraints, static_cast<size_t>(p));

    Matrix translation = Matrix::Zero(1, p), landmark = Matrix::Zero(1, p);
    translation.leftCols<2>() = ti.transpose();
    landmark.leftCols<2>() = target.transpose();
    Matrix direction =
        Matrix::Zero(QuadraticRangeFactor2::kDirectionRows, p);
    direction.leftCols<2>() = traits<Rot2>::QcqpValue<2>(u);

    Values Y;
    Y.insert(kTi, translation);
    Y.insert(kTarget, landmark);
    Y.insert(kU, direction);
    EXPECT_DOUBLES_EQUAL(expected, costs.error(Y), 1e-10);
  }
}

// A column dimension below the ambient dimension cannot represent the lift.
TEST(QuadraticRangeFactor, QcqpFactorsRejectTooFewColumns) {
  const QuadraticRangeFactor3 factor(kTi, kTarget, kU, 1.0, 1.0);
  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  CHECK_EXCEPTION(factor.qcqpFactors(&costs, &constraints, 2),
                  std::invalid_argument);
}

}  // namespace QcqpFixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
