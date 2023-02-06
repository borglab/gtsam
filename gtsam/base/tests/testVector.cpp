/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testVector.cpp
 * @brief  Unit tests for Vector class
 * @author Frank Dellaert
 **/

#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/testLie.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;

namespace {
  /* ************************************************************************* */
  template<typename Derived>
  Vector testFcn1(const Eigen::DenseBase<Derived>& in)
  {
    return in;
  }

  /* ************************************************************************* */
  template<typename Derived>
  Vector testFcn2(const Eigen::MatrixBase<Derived>& in)
  {
    return in;
  }
}

/* ************************************************************************* */
TEST(Vector, special_comma_initializer)
{
  Vector expected(3);
  expected(0) = 1;
  expected(1) = 2;
  expected(2) = 3;

  Vector actual1 = Vector3(1, 2, 3);
  Vector actual2(Vector3(1, 2, 3));

  Vector subvec1 = Vector2(2, 3);
  Vector actual4 = (Vector(3) << 1, subvec1).finished();

  Vector subvec2 = Vector2(1, 2);
  Vector actual5 = (Vector(3) << subvec2, 3).finished();

  Vector actual6 = testFcn1(Vector3(1, 2, 3));
  Vector actual7 = testFcn2(Vector3(1, 2, 3));

  EXPECT(assert_equal(expected, actual1));
  EXPECT(assert_equal(expected, actual2));
  EXPECT(assert_equal(expected, actual4));
  EXPECT(assert_equal(expected, actual5));
  EXPECT(assert_equal(expected, actual6));
  EXPECT(assert_equal(expected, actual7));
}

/* ************************************************************************* */
TEST(Vector, copy )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  double data[] = {10,20};
  Vector b(2);
  copy(data,data+2,b.data());
  EXPECT(assert_equal(a, b));
}

/* ************************************************************************* */
TEST(Vector, scalar_multiply )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  Vector b(2); b(0) = 1; b(1) = 2;
  EXPECT(assert_equal(a,b*10.0));
}

/* ************************************************************************* */
TEST(Vector, scalar_divide )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  Vector b(2); b(0) = 1; b(1) = 2;
  EXPECT(assert_equal(b,a/10.0));
}

/* ************************************************************************* */
TEST(Vector, negate )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  Vector b(2); b(0) = -10; b(1) = -20;
  EXPECT(assert_equal(b, -a));
}

/* ************************************************************************* */
TEST(Vector, householder )
{
  Vector x(4);
  x(0) = 3; x(1) = 1; x(2) = 5; x(3) = 1;

  Vector expected(4);
  expected(0) = 1.0; expected(1) = -0.333333; expected(2) = -1.66667; expected(3) = -0.333333;

  pair<double, Vector> result = house(x);

  EXPECT(result.first==0.5);
  EXPECT(equal_with_abs_tol(expected,result.second,1e-5));
}

/* ************************************************************************* */
TEST(Vector, concatVectors)
{
  Vector A(2);
  for(int i = 0; i < 2; i++)
    A(i) = i;
  Vector B(5);
  for(int i = 0; i < 5; i++)
    B(i) = i;

  Vector C(7);
  for(int i = 0; i < 2; i++) C(i) = A(i);
  for(int i = 0; i < 5; i++) C(i+2) = B(i);

  list<Vector> vs;
  vs.push_back(A);
  vs.push_back(B);
  Vector AB1 = concatVectors(vs);
  EXPECT(AB1 == C);

  Vector AB2 = concatVectors(2, &A, &B);
  EXPECT(AB2 == C);
}

/* ************************************************************************* */
TEST(Vector, weightedPseudoinverse )
{
  // column from a matrix
  Vector x(2);
  x(0) = 1.0; x(1) = 2.0;

  // create sigmas
  Vector sigmas(2);
  sigmas(0) = 0.1; sigmas(1) = 0.2;
  Vector weights = sigmas.array().square().inverse();

  // perform solve
  const auto [actual, precision] = weightedPseudoinverse(x, weights);

  // construct expected
  Vector expected(2);
  expected(0) = 0.5; expected(1) = 0.25;
  double expPrecision = 200.0;

  // verify
  EXPECT(assert_equal(expected,actual));
  EXPECT(std::abs(expPrecision-precision) < 1e-5);
}

/* ************************************************************************* */
TEST(Vector, weightedPseudoinverse_constraint )
{
  // column from a matrix
  Vector x(2);
  x(0) = 1.0; x(1) = 2.0;

  // create sigmas
  Vector sigmas(2);
  sigmas(0) = 0.0; sigmas(1) = 0.2;
  Vector weights = sigmas.array().square().inverse();
  // perform solve
  const auto [actual, precision] = weightedPseudoinverse(x, weights);

  // construct expected
  Vector expected(2);
  expected(0) = 1.0; expected(1) = 0.0;

  // verify
  EXPECT(assert_equal(expected,actual));
  EXPECT(std::isinf(precision));
}

/* ************************************************************************* */
TEST(Vector, weightedPseudoinverse_nan )
{
  Vector a = (Vector(4) << 1., 0., 0., 0.).finished();
  Vector sigmas = (Vector(4) << 0.1, 0.1, 0., 0.).finished();
  Vector weights = sigmas.array().square().inverse();
  const auto [pseudo, precision] = weightedPseudoinverse(a, weights);

  Vector expected = (Vector(4) << 1., 0., 0.,0.).finished();
  EXPECT(assert_equal(expected, pseudo));
  DOUBLES_EQUAL(100, precision, 1e-5);
}

/* ************************************************************************* */
TEST(Vector, dot )
{
  Vector a = Vector3(10., 20., 30.);
  Vector b = Vector3(2.0, 5.0, 6.0);
  DOUBLES_EQUAL(20+100+180,dot(a,b),1e-9);
}

/* ************************************************************************* */
TEST(Vector, axpy )
{
  Vector x = Vector3(10., 20., 30.);
  Vector y0 = Vector3(2.0, 5.0, 6.0);
  Vector y1 = y0, y2 = y0;
  y1 += 0.1 * x;
  y2.head(3) += 0.1 * x;
  Vector expected = Vector3(3.0, 7.0, 9.0);
  EXPECT(assert_equal(expected,y1));
  EXPECT(assert_equal(expected,Vector(y2)));
}

/* ************************************************************************* */
TEST(Vector, equals )
{
  Vector v1 = (Vector(1) << 0.0/std::numeric_limits<double>::quiet_NaN()).finished(); //testing nan
  Vector v2 = (Vector(1) << 1.0).finished();
  double tol = 1.;
  EXPECT(!equal_with_abs_tol(v1, v2, tol));
}

/* ************************************************************************* */
TEST(Vector, greater_than )
{
  Vector v1 = Vector3(1.0, 2.0, 3.0),
       v2 = Z_3x1;
  EXPECT(greaterThanOrEqual(v1, v1)); // test basic greater than
  EXPECT(greaterThanOrEqual(v1, v2)); // test equals
}

/* ************************************************************************* */
TEST(Vector, linear_dependent )
{
  Vector v1 = Vector3(1.0, 2.0, 3.0);
  Vector v2 = Vector3(-2.0, -4.0, -6.0);
  EXPECT(linear_dependent(v1, v2));
}

/* ************************************************************************* */
TEST(Vector, linear_dependent2 )
{
  Vector v1 = Vector3(0.0, 2.0, 0.0);
  Vector v2 = Vector3(0.0, -4.0, 0.0);
  EXPECT(linear_dependent(v1, v2));
}

/* ************************************************************************* */
TEST(Vector, linear_dependent3 )
{
  Vector v1 = Vector3(0.0, 2.0, 0.0);
  Vector v2 = Vector3(0.1, -4.1, 0.0);
  EXPECT(!linear_dependent(v1, v2));
}

//******************************************************************************
TEST(Vector, VectorIsVectorSpace) {
  GTSAM_CONCEPT_ASSERT1(IsVectorSpace<Vector5>);
  GTSAM_CONCEPT_ASSERT2(IsVectorSpace<Vector>);
}

TEST(Vector, RowVectorIsVectorSpace) {
  typedef Eigen::Matrix<double,1,-1> RowVector;
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<RowVector>);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
