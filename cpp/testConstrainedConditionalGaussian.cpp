/**
 * @file testConstrainedConditionalGaussian.cpp
 * @brief tests for constrained conditional gaussians
 * @author Alex Cunningham
 */


#include <CppUnitLite/TestHarness.h>
#include "ConstrainedConditionalGaussian.h"

using namespace gtsam;

/* ************************************************************************* */
TEST (ConstrainedConditionalGaussian, basic_unary1 )
{
	Vector v(2); v(0)=1.0; v(1)=2.0;

	// check unary constructor that doesn't require an R matrix
	// assumed identity matrix
	ConstrainedConditionalGaussian unary("x1",v);
	VectorConfig fg;
	fg.insert("x1", v);

	CHECK(assert_equal(v, unary.solve(fg)));
}

/* ************************************************************************* */
TEST (ConstrainedConditionalGaussian, basic_unary2 )
{
	Vector v(2); v(0)=1.0; v(1)=2.0;

	// check unary constructor that makes use of a A matrix
	Matrix A = eye(2) * 10;

	ConstrainedConditionalGaussian unary("x1",10*v, A);
	VectorConfig fg;
	fg.insert("x1", v);

	CHECK(assert_equal(v, unary.solve(fg)));
}

/* ************************************************************************* */
TEST (ConstrainedConditionalGaussian, basic_unary3 )
{
	Vector v(2); v(0)=1.0; v(1)=2.0;

	// check unary constructor that makes use of a A matrix
	// where A^(-1) exists, but A is not triangular
	Matrix A(2,2);
	A(0,0) = 1.0 ; A(0,1) = 2.0;
	A(1,0) = 2.0 ; A(1,1) = 1.0;

	Vector rhs = A*v;
	ConstrainedConditionalGaussian unary("x1",rhs, A);
	VectorConfig fg;
	fg.insert("x1", v);

	CHECK(assert_equal(v, unary.solve(fg)));
}

/* ************************************************************************* */
TEST (ConstrainedConditionalGaussian, basic_binary1 )
{
	// tests x = (A1^(-1))*(b - A2y) case, where A1 is invertible
	// A1 is not already triangular, however

	// RHS
	Vector b(2); b(0)=3.0; b(1)=4.0;

	// A1 - invertible
	Matrix A1(2,2);
	A1(0,0) = 1.0 ; A1(0,1) = 2.0;
	A1(1,0) = 2.0 ; A1(1,1) = 1.0;

	// A2 - not invertible - should still work
	Matrix A2(2,2);
	A2(0,0) = 1.0 ; A2(0,1) = 2.0;
	A2(1,0) = 2.0 ; A2(1,1) = 4.0;

	Vector y = Vector_(2, 1.0, 2.0);

	VectorConfig fg;
	fg.insert("x1", y);

	Vector expected = Vector_(2, -3.3333, 0.6667);

	ConstrainedConditionalGaussian binary("x2",b, A1, "x1", A2);

	CHECK(assert_equal(expected, binary.solve(fg), 1e-4));
}

/* ************************************************************************* */
TEST (ConstrainedConditionalGaussian, basic_ternary1 )
{
	// tests x = (A1^(-1))*(b - A2*y - A3*z) case, where A1 is invertible
	// A1 is not already triangular, however

	// RHS
	Vector b(2); b(0)=3.0; b(1)=4.0;

	// A1 - invertible
	Matrix A1(2,2);
	A1(0,0) = 1.0 ; A1(0,1) = 2.0;
	A1(1,0) = 2.0 ; A1(1,1) = 1.0;

	// A2 - not invertible - should still work
	Matrix A2(2,2);
	A2(0,0) = 1.0 ; A2(0,1) = 2.0;
	A2(1,0) = 2.0 ; A2(1,1) = 4.0;

	Matrix A3 = eye(2)*10;

	Vector y = Vector_(2, 1.0, 2.0);
	Vector z = Vector_(2, 1.0, -1.0);

	VectorConfig fg;
	fg.insert("x1", y);
	fg.insert("x2", z);

	Vector expected = Vector_(2, 6.6667, -9.3333);

	ConstrainedConditionalGaussian ternary("x3",b, A1, "x1", A2, "x2", A3);

	CHECK(assert_equal(expected, ternary.solve(fg), 1e-4));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

