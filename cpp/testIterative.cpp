/**
 *  @file   testIterative.cpp
 *  @brief  Unit tests for iterative methods
 *  @author Frank Dellaert
 **/

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "iterative.h"
#include "smallExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Iterative, steepestDescent )
{
	// Expected solution
	Ordering ord;
	ord += "l1", "x1", "x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
	VectorConfig expected = fg.optimize(ord); // destructive

	// Do gradient descent
	GaussianFactorGraph fg2 = createGaussianFactorGraph();
	VectorConfig zero = createZeroDelta();
	bool verbose = false;
	VectorConfig actual = steepestDescent(fg2, zero, verbose);
	CHECK(assert_equal(expected,actual,1e-2));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent )
{
	// Expected solution
	Ordering ord;
	ord += "l1", "x1", "x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
	VectorConfig expected = fg.optimize(ord); // destructive

	// create graph and get matrices
	GaussianFactorGraph fg2 = createGaussianFactorGraph();
	Matrix A;
	Vector b;
	Vector x0 = gtsam::zero(6);
	boost::tie(A, b) = fg2.matrix(ord);
	Vector expectedX = Vector_(6, -0.1, 0.1, -0.1, -0.1, 0.1, -0.2);

	// Do conjugate gradient descent, System version
	System Ab(A, b);
	Vector actualX = conjugateGradientDescent(Ab, x0);
	CHECK(assert_equal(expectedX,actualX,1e-9));

	// Do conjugate gradient descent, Matrix version
	Vector actualX2 = conjugateGradientDescent(A, b, x0);
	CHECK(assert_equal(expectedX,actualX2,1e-9));

	// Do conjugate gradient descent on factor graph
	VectorConfig zero = createZeroDelta();
	VectorConfig actual = conjugateGradientDescent(fg2, zero);
	CHECK(assert_equal(expected,actual,1e-2));

	// Test method
	VectorConfig actual2 = fg2.conjugateGradientDescent(zero);
	CHECK(assert_equal(expected,actual2,1e-2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
