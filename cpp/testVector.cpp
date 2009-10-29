/**
 * @file   testVector.cpp
 * @brief  Unit tests for Vector class
 * @author Frank Dellaert
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include "Vector.h"

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
	
  Vector AB = concatVectors(2, &A, &B);

  CHECK(AB == C);
}

/* ************************************************************************* */
TEST( TestVector, whouse_solve )
{
	// column from a matrix
	Vector x(2);
	x(0) = 1.0; x(1) = 2.0;

	// create precisions - correspond to sigmas = [0.1 0.2]
	Vector tau(2);
	tau(0) = 100; tau(1) = 25;

	// perform solve
	Vector act = whouse_solve(x, tau);

	// construct expected
	Vector exp(2);
	exp(0) = 0.5; exp(1) = 0.25;

	// verify
	CHECK(assert_equal(act, exp)); 
}

/* ************************************************************************* */
TEST( TestVector, whouse_subs_vector )
{
	// vector to update
	Vector b(2);
	b(0) = 5; b(1) = 6;

	// Vector to eliminate
	Vector a(2);
	a(0) = 1.0; a(1) = 2.0;

	Vector tau(2); //correspond to sigmas = [0.1 0.2]
	tau(0) = 100; tau(1) = 25;

	// find the pseudoinverse
	Vector pseudo = whouse_solve(a, tau);

	// substitute
	int row = 0; // eliminating the first column
	whouse_subs(b, row, a, pseudo);

	// create expected value
	Vector exp(2);
	exp(0) = 5; exp(1) = -2.0;

	// verify
	CHECK(assert_equal(b, exp, 1e-5));
}

/* ************************************************************************* */
TEST( TestVector, whouse_subs_vector2 )
{
	double sigma1 = 0.2; double tau1 = 1/(sigma1*sigma1);
	double sigma2 = 0.1; double tau2 = 1/(sigma2*sigma2);
	Vector sigmas = Vector_(4, sigma1, sigma1, sigma2, sigma2);

	Vector a1 = Vector_(4, -1., 0., 1., 0.);
	Vector tau = Vector_(4, tau1, tau1, tau2, tau2);
	Vector pseudo1 = whouse_solve(a1, tau);

	Vector expected = Vector_(4,-0.2, 0., 0.8, 0.);
	CHECK(assert_equal(pseudo1, expected, 1e-4));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
