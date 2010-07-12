/**
 * @file testConstraintOptimizer.cpp
 * @brief Tests the optimization engine for SQP and BFGS Quadratic programming techniques
 * @author Alex Cunningham
 */

#include <iostream>
#include <limits>

#include <boost/tuple/tuple.hpp>
#include <boost/optional.hpp>

#include <CppUnitLite/TestHarness.h>

#include <Ordering.h>
#include <ConstraintOptimizer.h>

#define GTSAM_MAGIC_KEY

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;

#include <smallExample.h>
using namespace example;

/* ************************************************************************* */
TEST( matrix, unconstrained_fg_ata ) {
	// create a graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	Matrix A; Vector b;
	Ordering ordering;
	ordering += Symbol('l', 1), Symbol('x', 1), Symbol('x', 2);
	boost::tie(A, b) = fg.matrix(ordering);
	Matrix B_ata = prod(trans(A), A);

	// solve subproblem
	Vector actual = solve_ldl(B_ata, prod(trans(A), b));

	// verify
	Vector expected = createCorrectDelta().vector();
	CHECK(assert_equal(expected,actual));
}

///* ************************************************************************* */
//TEST( matrix, unconstrained_fg ) {
//	// create a graph
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//
//	Matrix A; Vector b;
//	Ordering ordering;
//	ordering += Symbol('l', 1), Symbol('x', 1), Symbol('x', 2);
//	boost::tie(A, b) = fg.matrix(ordering);
//	Matrix B_ata = prod(trans(A), A);
////	print(B_ata, "B_ata");
////	print(b, "  b");
//
//	// parameters
//	size_t maxIt = 50;
//	double stepsize = 0.1;
//
//	// iterate to solve
//	VectorConfig x = createZeroDelta();
//	BFGSEstimator B(x.dim());
//
//	Vector step;
//
//	for (size_t i=0; i<maxIt; ++i) {
////		cout << "Error at Iteration: " << i << " is " << fg.error(x) << endl;
//
//		// find the gradient
//		Vector dfx = fg.gradient(x).vector();
////		print(dfx, "   dfx");
//		CHECK(assert_equal(-1.0 * prod(trans(A), b - A*x.vector()), dfx));
//
//		// update hessian
//	    if (i>0) {
//	    	B.update(dfx, step);
//	    } else {
//	    	B.update(dfx);
//	    }
//
//	    // solve subproblem
////	    print(B.getB(), " B_bfgs");
//	    Vector delta = solve_ldl(B.getB(), -dfx);
////	    Vector delta = solve_ldl(B_ata, -dfx);
//
////	    print(delta, "   delta");
//
//	    // update
//		step = stepsize * delta;
////	    step = linesearch(x, delta, penalty); // TODO: switch here
//	    x = expmap(x, step);
////	    print(step, "   step");
//	}
//
//	// verify
//	VectorConfig expected = createCorrectDelta();
//	CHECK(assert_equal(expected,x, 1e-4));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
