/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testLinearEquality.cpp
 *  @brief  Unit tests for LinearEquality
 *  @author Duy-Nguyen Ta
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam_unstable/linear/LinearEquality.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(LinearEquality)

namespace {
namespace simple {
// Terms we'll use
const vector<pair<Key, Matrix> > terms{
    make_pair(5, I_3x3), make_pair(10, 2 * I_3x3), make_pair(15, 3 * I_3x3)};

// RHS and sigmas
const Vector b = (Vector(3) << 1., 2., 3.).finished();
const SharedDiagonal noise = noiseModel::Constrained::All(3);
}  // namespace simple
}  // namespace

/* ************************************************************************* */
TEST(LinearEquality, constructors_and_accessors) {
  using namespace simple;

  // Test for using different numbers of terms
  {
    // One term constructor
    LinearEquality expected(terms.begin(), terms.begin() + 1, b, 0);
    LinearEquality actual(terms[0].first, terms[0].second, b, 0);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[0].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[0].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(assert_equal(*noise, *actual.get_model()));
  }
  {
    // Two term constructor
    LinearEquality expected(terms.begin(), terms.begin() + 2, b, 0);
    LinearEquality actual(terms[0].first, terms[0].second, terms[1].first,
                          terms[1].second, b, 0);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[1].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[1].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(assert_equal(*noise, *actual.get_model()));
  }
  {
    // Three term constructor
    LinearEquality expected(terms.begin(), terms.begin() + 3, b, 0);
    LinearEquality actual(terms[0].first, terms[0].second, terms[1].first,
                          terms[1].second, terms[2].first, terms[2].second, b,
                          0);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[2].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[2].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(assert_equal(*noise, *actual.get_model()));
  }
}

/* ************************************************************************* */
TEST(LinearEquality, Hessian_conversion) {
  HessianFactor hessian(
      0,
      (Matrix(4, 4) << 1.57, 2.695, -1.1, -2.35, 2.695, 11.3125, -0.65, -10.225,
       -1.1, -0.65, 1, 0.5, -2.35, -10.225, 0.5, 9.25)
          .finished(),
      (Vector(4) << -7.885, -28.5175, 2.75, 25.675).finished(), 73.1725);

  try {
    LinearEquality actual(hessian);
    EXPECT(false);
  } catch (const std::runtime_error& exception) {
    EXPECT(true);
  }
}

/* ************************************************************************* */
TEST(LinearEquality, error) {
  LinearEquality factor(simple::terms, simple::b, 0);

  VectorValues values;
  values.insert(5, Vector::Constant(3, 1.0));
  values.insert(10, Vector::Constant(3, 0.5));
  values.insert(15, Vector::Constant(3, 1.0 / 3.0));

  Vector expected_unwhitened(3);
  expected_unwhitened << 2.0, 1.0, 0.0;
  Vector actual_unwhitened = factor.unweighted_error(values);
  EXPECT(assert_equal(expected_unwhitened, actual_unwhitened));

  // whitened is meaningless in constraints
  Vector expected_whitened(3);
  expected_whitened = expected_unwhitened;
  Vector actual_whitened = factor.error_vector(values);
  EXPECT(assert_equal(expected_whitened, actual_whitened));

  double expected_error = 0.0;
  double actual_error = factor.error(values);
  DOUBLES_EQUAL(expected_error, actual_error, 1e-10);
}

/* ************************************************************************* */
TEST(LinearEquality, matrices_NULL) {
  // Make sure everything works with nullptr noise model
  LinearEquality factor(simple::terms, simple::b, 0);

  Matrix AExpected(3, 9);
  AExpected << simple::terms[0].second, simple::terms[1].second,
      simple::terms[2].second;
  Vector rhsExpected = simple::b;
  Matrix augmentedJacobianExpected(3, 10);
  augmentedJacobianExpected << AExpected, rhsExpected;

  // Whitened Jacobian
  EXPECT(assert_equal(AExpected, factor.jacobian().first));
  EXPECT(assert_equal(rhsExpected, factor.jacobian().second));
  EXPECT(assert_equal(augmentedJacobianExpected, factor.augmentedJacobian()));

  // Unwhitened Jacobian
  EXPECT(assert_equal(AExpected, factor.jacobianUnweighted().first));
  EXPECT(assert_equal(rhsExpected, factor.jacobianUnweighted().second));
  EXPECT(assert_equal(augmentedJacobianExpected,
                      factor.augmentedJacobianUnweighted()));
}

/* ************************************************************************* */
TEST(LinearEquality, matrices) {
  // And now witgh a non-unit noise model
  LinearEquality factor(simple::terms, simple::b, 0);

  Matrix jacobianExpected(3, 9);
  jacobianExpected << simple::terms[0].second, simple::terms[1].second,
      simple::terms[2].second;
  Vector rhsExpected = simple::b;
  Matrix augmentedJacobianExpected(3, 10);
  augmentedJacobianExpected << jacobianExpected, rhsExpected;

  Matrix augmentedHessianExpected =
      augmentedJacobianExpected.transpose() * simple::noise->R().transpose() *
      simple::noise->R() * augmentedJacobianExpected;

  // Whitened Jacobian
  EXPECT(assert_equal(jacobianExpected, factor.jacobian().first));
  EXPECT(assert_equal(rhsExpected, factor.jacobian().second));
  EXPECT(assert_equal(augmentedJacobianExpected, factor.augmentedJacobian()));

  // Unwhitened Jacobian
  EXPECT(assert_equal(jacobianExpected, factor.jacobianUnweighted().first));
  EXPECT(assert_equal(rhsExpected, factor.jacobianUnweighted().second));
  EXPECT(assert_equal(augmentedJacobianExpected,
                      factor.augmentedJacobianUnweighted()));
}

/* ************************************************************************* */
TEST(LinearEquality, operators) {
  Matrix I = I_2x2;
  Vector b = (Vector(2) << 0.2, -0.1).finished();
  LinearEquality lf(1, -I, 2, I, b, 0);

  VectorValues c;
  c.insert(1, (Vector(2) << 10., 20.).finished());
  c.insert(2, (Vector(2) << 30., 60.).finished());

  // test A*x
  Vector expectedE = (Vector(2) << 20., 40.).finished();
  Vector actualE = lf * c;
  EXPECT(assert_equal(expectedE, actualE));

  // test A^e
  VectorValues expectedX;
  expectedX.insert(1, (Vector(2) << -20., -40.).finished());
  expectedX.insert(2, (Vector(2) << 20., 40.).finished());
  VectorValues actualX = VectorValues::Zero(expectedX);
  lf.transposeMultiplyAdd(1.0, actualE, actualX);
  EXPECT(assert_equal(expectedX, actualX));

  // test gradient at zero
  Matrix A;
  Vector b2;
  std::tie(A, b2) = lf.jacobian();
  VectorValues expectedG;
  expectedG.insert(1, (Vector(2) << 0.2, -0.1).finished());
  expectedG.insert(2, (Vector(2) << -0.2, 0.1).finished());
  VectorValues actualG = lf.gradientAtZero();
  EXPECT(assert_equal(expectedG, actualG));
}

/* ************************************************************************* */
TEST(LinearEquality, default_error) {
  LinearEquality f;
  double actual = f.error(VectorValues());
  DOUBLES_EQUAL(0.0, actual, 1e-15);
}

//* ************************************************************************* */
TEST(LinearEquality, empty) {
  // create an empty factor
  LinearEquality f;
  EXPECT(f.empty());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
