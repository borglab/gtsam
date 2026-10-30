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
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/AugmentedPoint.h>

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

// Verifies the lifted coordinates and both quadratic equalities.
TEST(AugmentedPoint, QcqpLiftAndConstraints3D) {
  const Point point(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Matrix X = Traits::QcqpValue<1>(point);

  EXPECT_LONGS_EQUAL(5, X.rows());
  EXPECT_LONGS_EQUAL(1, X.cols());
  const Vector expected = (Vector(5) << 1.0, 1.0, -2.0, 3.0, 14.0).finished();
  EXPECT(expected.isApprox(X.col(0), 1e-12));

  const auto constraints = Traits::QcqpConstraints<1>();
  EXPECT_LONGS_EQUAL(2, constraints.size());
  if (constraints.size() != 2) return;

  // Each tuple represents X^T A X + 2 a^T X = b.
  for (const auto& constraint : constraints) {
    const Matrix& A = std::get<0>(constraint);
    const Vector& a = std::get<1>(constraint);
    const double b = std::get<2>(constraint);

    EXPECT_LONGS_EQUAL(5, A.rows());
    EXPECT_LONGS_EQUAL(5, A.cols());
    EXPECT_LONGS_EQUAL(0, a.size());
    EXPECT(A.isApprox(A.transpose(), 1e-12));
    EXPECT_DOUBLES_EQUAL(b, (X.transpose() * A * X)(0, 0), 1e-12);
  }

  const Matrix& A = std::get<0>(constraints.at(0));
  const Matrix& H = std::get<0>(constraints.at(1));
  EXPECT_DOUBLES_EQUAL(0.0, std::get<2>(constraints.at(0)), 1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, std::get<2>(constraints.at(1)), 1e-12);
  EXPECT_DOUBLES_EQUAL(-0.5, A(0, 4), 1e-12);
  EXPECT_DOUBLES_EQUAL(-0.5, A(4, 0), 1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, H(0, 0), 1e-12);

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

  // h^2 = 1 permits h = -1; recovery must cancel the global sign.
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
