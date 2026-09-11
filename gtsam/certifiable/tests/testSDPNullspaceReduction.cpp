/* Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/certifiable/SDPNullspaceReduction.h>

using namespace gtsam;

/* ************************************************************************* */
namespace nullspace_fixture {

const std::map<Key, DenseIndex> dimensions{{0, 3}, {1, 3}};

QcqpProblem Problem(bool bothHomogeneous, bool coupledAnchor = false) {
  NonlinearEqualityConstraints constraints;
  InsertQcqpConstraints<Vector2, 1>(0, &constraints);
  if (bothHomogeneous) InsertQcqpConstraints<Vector2, 1>(1, &constraints);
  Matrix A{{1, 2, 0}, {0, 0, 1}};
  if (coupledAnchor) A(0, 2) = 1;
  constraints.push_back(
      LinearConstraint::Equal(JacobianFactor(0, A, Vector2(5, -3)))
          .createEqualityFactor());
  return QcqpProblem(NonlinearFactorGraph(), constraints);
}

// Duplicate h rows and nonzero fixed coordinates are exact substitutions.
TEST(SDPNullspaceReduction, SubstitutionAndObjectiveParity) {
  const auto bases = internal::EliminateKnownNullDirections(
      Problem(true), dimensions, {{0, 1}});
  const Matrix B = bases.at({0, 1});
  const Matrix expected{{1, 0, 0}, {2, 0, 0}, {-3, 0, 0},
                        {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  EXPECT(assert_equal(expected, B));
  const Matrix Z{{2, 1, 0}, {1, 3, 1}, {0, 1, 2}};
  const Matrix Q = Matrix::Identity(6, 6) + Matrix::Ones(6, 6);
  EXPECT_DOUBLES_EQUAL((Q * B * Z * B.transpose()).trace(),
                       (B.transpose() * Q * B * Z).trace(), 1e-12);
}

// No homogeneous-coordinate reduction is inferred without h^2=1 constraints.
TEST(SDPNullspaceReduction, MissingNormalizationLeavesCliqueUnchanged) {
  const auto bases = internal::EliminateKnownNullDirections(
      Problem(false), dimensions, {{0, 1}});
  EXPECT(assert_equal(Matrix(Matrix::Identity(6, 6)), bases.at({0, 1})));
}

// A coupled anchor is retained; only its separate fixed coordinate is removed.
TEST(SDPNullspaceReduction, CoupledEquationIsNotGuessed) {
  const auto bases = internal::EliminateKnownNullDirections(
      Problem(true, true), dimensions, {{0, 1}, {1}});
  EXPECT_LONGS_EQUAL(4, bases.at({0, 1}).cols());
  EXPECT(assert_equal(Matrix(Matrix::Identity(3, 3)), bases.at({1})));
}

}  // namespace nullspace_fixture
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
