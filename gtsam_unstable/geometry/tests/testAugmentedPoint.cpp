/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */

/**
 * @file testAugmentedPoint.cpp
 * @brief Tests for AugmentedPoint vector-space operations and QCQP traits.
 * @author Oluwatodimu Afolayan
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam_unstable/geometry/AugmentedPoint.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

using namespace gtsam;

/* ************************************************************************* */
namespace augmented_point3_fixture {

using Point = AugmentedPoint<3>;
using Traits = traits<Point>;

// Dynamic wrapper inputs must be validated before conversion to fixed storage.
TEST(AugmentedPoint, DynamicCoordinates) {
  const Vector coordinates = (Vector(3) << 1.0, -2.0, 3.0).finished();
  EXPECT(Point(coordinates).vector().isApprox(coordinates));
  CHECK_EXCEPTION(Point(Vector::Zero(2)), std::invalid_argument);
  CHECK_EXCEPTION(Point(Vector::Zero(4)), std::invalid_argument);
  CHECK_EXCEPTION(Point(Matrix::Zero(1, 3)), std::invalid_argument);
}

// Verifies the additive group operations and vector-space traits.
TEST(AugmentedPoint, VectorSpace) {
  const Point origin = Point::Identity();
  const Point point(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Point other(Eigen::Vector3d(4.0, 5.0, 6.0));

  // Compare Eigen vectors directly: avoids ambiguity between GTSAM's
  // assert_equal(Vector, Vector) and assert_equal(Matrix, Matrix) overloads.
  EXPECT(origin.vector().isZero(1e-12));
  EXPECT((point + other)
             .vector()
             .isApprox(point.vector() + other.vector(), 1e-12));
  EXPECT((point - other)
             .vector()
             .isApprox(point.vector() - other.vector(), 1e-12));
  EXPECT((-point).vector().isApprox(-point.vector(), 1e-12));
  EXPECT((2.0 * point).vector().isApprox(2.0 * point.vector(), 1e-12));
  EXPECT((point * 2.0).vector().isApprox(2.0 * point.vector(), 1e-12));

  const Vector tangent = Vector::Ones(3);
  const Point shifted = point + tangent;
  EXPECT(shifted.vector().isApprox(point.vector() + tangent, 1e-12));
  EXPECT(Traits::Local(point, shifted).isApprox(tangent, 1e-12));
  EXPECT(Traits::Retract(point, tangent).equals(shifted));

  CHECK_EXCEPTION(point + Vector::Zero(2), std::invalid_argument);
}

// Verifies the lifted coordinates and the three equalities.
TEST(AugmentedPoint, QcqpLiftAndConstraints3D) {
  const Point point(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Matrix X = Traits::QcqpValue<1>(point);

  EXPECT_LONGS_EQUAL(5, X.rows());
  EXPECT_LONGS_EQUAL(1, X.cols());
  const Vector expected = (Vector(5) << 1.0, 1.0, -2.0, 3.0, 14.0).finished();
  EXPECT(expected.isApprox(X.col(0), 1e-12));

  const auto constraints = Traits::QcqpConstraints<1>();
  EXPECT_LONGS_EQUAL(3, constraints.size());
  if (constraints.size() != 3) return;

  // Each tuple represents X^T A X + 2 a^T X = b.
  for (const auto& constraint : constraints) {
    const Matrix& A = std::get<0>(constraint);
    const Vector& a = std::get<1>(constraint);
    const double b = std::get<2>(constraint);

    EXPECT_LONGS_EQUAL(5, A.rows());
    EXPECT_LONGS_EQUAL(5, A.cols());
    EXPECT(A.isApprox(A.transpose(), 1e-12));
    const double linear = a.size() == 0 ? 0.0 : 2.0 * a.dot(X.col(0));
    EXPECT_DOUBLES_EQUAL(b, (X.transpose() * A * X)(0, 0) + linear, 1e-12);
  }

  // x^T x - h*u = 0 and h^2 = 1 are quadratic; h = 1 is linear.
  const Matrix& A = std::get<0>(constraints.at(0));
  const Matrix& H = std::get<0>(constraints.at(1));
  EXPECT_LONGS_EQUAL(0, std::get<1>(constraints.at(0)).size());
  EXPECT_LONGS_EQUAL(0, std::get<1>(constraints.at(1)).size());
  EXPECT_DOUBLES_EQUAL(0.0, std::get<2>(constraints.at(0)), 1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, std::get<2>(constraints.at(1)), 1e-12);
  EXPECT_DOUBLES_EQUAL(-0.5, A(0, 4), 1e-12);
  EXPECT_DOUBLES_EQUAL(-0.5, A(4, 0), 1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, H(0, 0), 1e-12);

  const auto& [L, a, b] = constraints.at(2);
  const Vector expectedLinear = (Vector(5) << 0.5, 0.0, 0.0, 0.0, 0.0).finished();
  EXPECT(L.isZero(0.0));
  EXPECT(assert_equal(expectedLinear, a));
  EXPECT_DOUBLES_EQUAL(1.0, b, 1e-12);

  // Changing u violates x^T x - h*u = 0.
  Matrix inconsistent = X;
  inconsistent(4, 0) += 1.0;
  EXPECT(std::abs((inconsistent.transpose() * A * inconsistent)(0, 0)) > 1e-9);
}

// Verifies round-trip recovery and global sign invariance.
TEST(AugmentedPoint, QcqpRoundTripAndSign) {
  const Point point(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Matrix X = qcqpValue<3>(point);
  EXPECT(point.equals(fromQcqpValue<3>(X)));

  // Recovery divides by h, so a negated lift gives the same point.
  EXPECT(point.equals(fromQcqpValue<3>(-X)));

  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(Matrix::Zero(4, 1)),
                  std::invalid_argument);
  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(Matrix::Zero(5, 2)),
                  std::invalid_argument);
  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(Matrix::Zero(5, 1)),
                  std::invalid_argument);

  Matrix invalidH = X;
  invalidH(0, 0) = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(invalidH), std::invalid_argument);
}

// Regression: with only x^T x = h*u and h^2 = 1, a lift with h = -1 was
// feasible. A cost that reads the raw coordinates x of two points, here a
// relative position x2 - x1 = delta, then had a zero-cost feasible solution
// whose recovered points x/h violate that relative position. The variable
// constraints alone must now reject it, without an added h = 1 constraint.
TEST(AugmentedPoint, NegativeHomogeneousCoordinateIsInfeasible) {
  const Key key1 = 1, key2 = 2;
  const Point p1(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Eigen::Vector3d delta(0.5, 0.25, -1.0);

  QcqpVariableConstraints variableConstraints;
  variableConstraints.add(key1, Traits::QcqpConstraints<1>);
  variableConstraints.add(key2, Traits::QcqpConstraints<1>);
  EXPECT_LONGS_EQUAL(6, variableConstraints.constraints().size());

  // QCQP cost 0.5 |x2 - x1 - delta|^2 on the raw lifted coordinates.
  Matrix S = Matrix::Zero(3, 5);
  S.block<3, 3>(0, 1).setIdentity();
  NonlinearFactorGraph costs;
  costs.emplace_shared<QpCost>(
      JacobianFactor(key1, -S, key2, S, delta, noiseModel::Unit::Create(3)));

  // The intended solution: canonical lifts of p1 and p1 + delta.
  const Point p2(p1.vector() + delta);
  Values canonical;
  canonical.insert(key1, Traits::QcqpValue<1>(p1));
  canonical.insert(key2, Traits::QcqpValue<1>(p2));
  EXPECT_DOUBLES_EQUAL(0.0, costs.error(canonical), 1e-12);
  EXPECT_DOUBLES_EQUAL(
      0.0, variableConstraints.constraints().violationNorm(canonical), 1e-12);

  // The spurious solution: key2 holds the negated lift of q = -(p1 + delta),
  // so its raw coordinates are p1 + delta and the cost is still zero.
  const Point q(-p2.vector());
  Values spurious;
  spurious.insert(key1, Traits::QcqpValue<1>(p1));
  spurious.insert(key2, Matrix(-Traits::QcqpValue<1>(q)));
  EXPECT_DOUBLES_EQUAL(0.0, costs.error(spurious), 1e-12);

  // It satisfies x^T x = h*u and h^2 = 1 but recovers a point other than p2.
  const Matrix X2 = spurious.at<Matrix>(key2);
  const auto constraints = Traits::QcqpConstraints<1>();
  for (size_t i = 0; i < 2; ++i) {
    const auto& [A, a, b] = constraints.at(i);
    EXPECT_DOUBLES_EQUAL(b, (X2.transpose() * A * X2)(0, 0), 1e-12);
  }
  EXPECT(Traits::FromQcqpValue<1>(X2).equals(q));
  EXPECT(!q.equals(p2, 1e-3));

  // h = 1 excludes it.
  EXPECT(variableConstraints.constraints().violationNorm(spurious) > 1.0);
}

// Verifies unsupported multi-column QCQP conversions are rejected.
TEST(AugmentedPoint, RejectsUnsupportedQcqpColumns) {
  const Point point(Eigen::Vector3d(1.0, 2.0, 3.0));

  CHECK_EXCEPTION(Traits::QcqpValue<2>(point), std::invalid_argument);
  CHECK_EXCEPTION(Traits::QcqpConstraints<2>(), std::invalid_argument);
  CHECK_EXCEPTION(Traits::FromQcqpValue<2>(Matrix::Zero(5, 1)),
                  std::invalid_argument);
}

}  // namespace augmented_point3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace augmented_point5_fixture {

// Dimensions beyond the explicit instantiations remain usable in C++.
TEST(AugmentedPoint, ArbitraryDimension) {
  const AugmentedPoint<5> point(Vector::Ones(5));
  const Matrix lifted = qcqpValue(point);
  EXPECT_LONGS_EQUAL(5, traits<AugmentedPoint<5>>::dimension);
  EXPECT_LONGS_EQUAL(7, lifted.rows());
  EXPECT_DOUBLES_EQUAL(5.0, lifted(6, 0), 1e-12);
  EXPECT(point.equals(fromQcqpValue<5>(-lifted)));
}

}  // namespace augmented_point5_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
