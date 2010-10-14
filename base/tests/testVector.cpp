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

#include <iostream>
#include <gtsam/CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <gtsam/base/Vector.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( TestVector, Vector_variants )
{
  Vector a = Vector_(2,10.0,20.0);
  double data[] = {10,20};
  Vector b = Vector_(2,data);
  CHECK(a==b);
}

/* ************************************************************************* */
TEST( TestVector, copy )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  double data[] = {10,20};
  Vector b(2); copy(data,data+2,b.begin());
  CHECK(a==b);
}

/* ************************************************************************* */
TEST( TestVector, zero1 )
{
  Vector v(2,0.0);
  CHECK(zero(v)==true);
}

/* ************************************************************************* */
TEST( TestVector, zero2 )
{
  Vector a = zero(2);
  Vector b(2,0.0);
  CHECK(a==b);
}

/* ************************************************************************* */
TEST( TestVector, scalar_multiply )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  Vector b(2); b(0) = 1; b(1) = 2;
  CHECK(a==b*10.0);
}

/* ************************************************************************* */
TEST( TestVector, scalar_divide )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  Vector b(2); b(0) = 1; b(1) = 2;
  CHECK(b==a/10.0);
}

/* ************************************************************************* */
TEST( TestVector, negate )
{
  Vector a(2); a(0) = 10; a(1) = 20;
  Vector b(2); b(0) = -10; b(1) = -20;
  CHECK(b==-a);
}

/* ************************************************************************* */
TEST( TestVector, sub )
{
  Vector a(6);
  a(0) = 10; a(1) = 20; a(2) = 3; 
  a(3) = 34; a(4) = 11; a(5) = 2;
  
  Vector result(sub(a,2,5));
  
  Vector b(3);
  b(0) = 3; b(1) = 34; b(2) =11; 

  CHECK(b==result);
}

/* ************************************************************************* */
TEST( TestVector, subInsert )
{
	Vector big = zero(6),
		   small = ones(3);

	size_t i = 2;
	subInsert(big, small, i);

	Vector expected = Vector_(6, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0);

	CHECK(assert_equal(expected, big));
}

/* ************************************************************************* */
TEST( TestVector, householder )
{
  Vector x(4); 
  x(0) = 3; x(1) = 1; x(2) = 5; x(3) = 1;
  
  Vector expected(4);
  expected(0) = 1.0; expected(1) = -0.333333; expected(2) = -1.66667; expected(3) = -0.333333;
  
  pair<double, Vector> result = house(x);
  
  CHECK(result.first==0.5);
  CHECK(equal_with_abs_tol(expected,result.second,1e-5));
}

/* ************************************************************************* */
TEST( TestVector, zeros )
{
  Vector a(2); a(0) = 0; a(1) = 0;
  Vector b(2,0.0);
  CHECK(b==a);
}

/* ************************************************************************* */
TEST( TestVector, concatVectors)
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
  CHECK(AB1 == C);

  Vector AB2 = concatVectors(2, &A, &B);
  CHECK(AB2 == C);
}

/* ************************************************************************* */
TEST( TestVector, weightedPseudoinverse )
{
	// column from a matrix
	Vector x(2);
	x(0) = 1.0; x(1) = 2.0;

	// create sigmas
	Vector sigmas(2);
	sigmas(0) = 0.1; sigmas(1) = 0.2;
	Vector weights = reciprocal(emul(sigmas,sigmas));

	// perform solve
	Vector actual; double precision;
	boost::tie(actual, precision) = weightedPseudoinverse(x, weights);

	// construct expected
	Vector expected(2);
	expected(0) = 0.5; expected(1) = 0.25;
	double expPrecision = 200.0;

	// verify
	CHECK(assert_equal(expected,actual));
	CHECK(fabs(expPrecision-precision) < 1e-5);
}

/* ************************************************************************* */
TEST( TestVector, weightedPseudoinverse_constraint )
{
	// column from a matrix
	Vector x(2);
	x(0) = 1.0; x(1) = 2.0;

	// create sigmas
	Vector sigmas(2);
	sigmas(0) = 0.0; sigmas(1) = 0.2;
	Vector weights = reciprocal(emul(sigmas,sigmas));

	// perform solve
	Vector actual; double precision;
	boost::tie(actual, precision) = weightedPseudoinverse(x, weights);

	// construct expected
	Vector expected(2);
	expected(0) = 1.0; expected(1) = 0.0;

	// verify
	CHECK(assert_equal(expected,actual));
	CHECK(isinf(precision));
}

/* ************************************************************************* */
TEST( TestVector, weightedPseudoinverse_nan )
{
	Vector a = Vector_(4, 1., 0., 0., 0.);
	Vector sigmas = Vector_(4, 0.1, 0.1, 0., 0.);
	Vector weights = reciprocal(emul(sigmas,sigmas));
	Vector pseudo; double precision;
	boost::tie(pseudo, precision) = weightedPseudoinverse(a, weights);

	Vector expected = Vector_(4, 1., 0., 0.,0.);
	CHECK(assert_equal(expected, pseudo));
	DOUBLES_EQUAL(100, precision, 1e-5);
}


/* ************************************************************************* */
TEST( TestVector, ediv )
{
  Vector a = Vector_(3,10.,20.,30.);
  Vector b = Vector_(3,2.0,5.0,6.0);
  Vector actual(ediv(a,b));

  Vector c = Vector_(3,5.0,4.0,5.0);
  CHECK(assert_equal(c,actual));
}

/* ************************************************************************* */
TEST( TestVector, dot )
{
  Vector a = Vector_(3,10.,20.,30.);
  Vector b = Vector_(3,2.0,5.0,6.0);
  DOUBLES_EQUAL(20+100+180,dot(a,b),1e-9);
}

/* ************************************************************************* */
TEST( TestVector, axpy )
{
  Vector x = Vector_(3,10.,20.,30.);
  Vector y = Vector_(3,2.0,5.0,6.0);
  axpy(0.1,x,y);
  Vector expected = Vector_(3,3.0,7.0,9.0);
  CHECK(assert_equal(expected,y));
}

/* ************************************************************************* */
TEST( TestVector, equals )
{
	Vector v1 = Vector_(1, 0.0/0.0); //testing nan
	Vector v2 = Vector_(1, 1.0);
	double tol = 1.;
	CHECK(!equal_with_abs_tol(v1, v2, tol));
}

/* ************************************************************************* */
TEST( TestVector, greater_than )
{
	Vector v1 = Vector_(3, 1.0, 2.0, 3.0),
		   v2 = zero(3);
	CHECK(greaterThanOrEqual(v1, v1)); // test basic greater than
	CHECK(greaterThanOrEqual(v1, v2)); // test equals
}

/* ************************************************************************* */
TEST( TestVector, reciprocal )
{
	Vector v = Vector_(3, 1.0, 2.0, 4.0);
  CHECK(assert_equal(Vector_(3, 1.0, 0.5, 0.25),reciprocal(v)));
}

/* ************************************************************************* */
TEST( TestVector, linear_dependent )
{
	Vector v1 = Vector_(3, 1.0, 2.0, 3.0);
	Vector v2 = Vector_(3, -2.0, -4.0, -6.0);
	CHECK(linear_dependent(v1, v2));
}

/* ************************************************************************* */
TEST( TestVector, linear_dependent2 )
{
	Vector v1 = Vector_(3, 0.0, 2.0, 0.0);
	Vector v2 = Vector_(3, 0.0, -4.0, 0.0);
	CHECK(linear_dependent(v1, v2));
}

/* ************************************************************************* */
TEST( TestVector, linear_dependent3 )
{
	Vector v1 = Vector_(3, 0.0, 2.0, 0.0);
	Vector v2 = Vector_(3, 0.1, -4.1, 0.0);
	CHECK(!linear_dependent(v1, v2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
