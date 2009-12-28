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
TEST( Iterative, gradientDescent )
{
	// Expected solution
	Ordering ord;
  ord += "l1","x1","x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
  VectorConfig expected = fg.optimize(ord); // destructive

  // Do gradient descent
  GaussianFactorGraph fg2 = createGaussianFactorGraph();
  VectorConfig zero = createZeroDelta();
	VectorConfig actual = fg2.gradientDescent(zero);
	CHECK(assert_equal(expected,actual,1e-2));

  // Do conjugate gradient descent
	//VectorConfig actual2 = fg2.conjugateGradientDescent(zero);
	VectorConfig actual2 = conjugateGradientDescent(fg2,zero);
	CHECK(assert_equal(expected,actual2,1e-2));

	// Do conjugate gradient descent, Matrix version
	Matrix A;Vector b;
	boost::tie(A,b) = fg2.matrix(ord);
//	print(A,"A");
//	print(b,"b");
	Vector x0 = gtsam::zero(6);
	Vector actualX = conjugateGradientDescent(A,b,x0);
	Vector expectedX = Vector_(6, -0.1, 0.1, -0.1, -0.1, 0.1, -0.2);
	CHECK(assert_equal(expectedX,actualX,1e-9));

	// Do conjugate gradient descent, System version
	System Ab = make_pair(A,b);
	Vector actualX2 = conjugateGradientDescent(Ab,x0);
	CHECK(assert_equal(expectedX,actualX2,1e-9));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
