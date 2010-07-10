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
#include <smallExample.h>

#define GTSAM_MAGIC_KEY

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* */
// Example of a single Constrained QP problem from the matlab testCQP.m file.
TEST( matrix, CQP_example ) {

	Matrix A = Matrix_(3, 2,
			-1.0,  -1.0,
			-2.0,   1.0,
			 1.0,  -1.0);
	Matrix At = trans(A),
		   B = 2.0 * eye(3,3);

	Vector b = Vector_(2, 4.0, -2.0),
	 	   g = zero(3);

	Matrix G = zeros(5,5);
	insertSub(G, B, 0, 0);
	insertSub(G, A, 0, 3);
	insertSub(G, At, 3, 0);

	Vector rhs = zero(5);
	subInsert(rhs, -1.0*g, 0);
	subInsert(rhs, -1.0*b, 3);

	// solve the system with the LDL solver
	Vector actualFull = solve_ldl(G, rhs);
	Vector actual = sub(actualFull, 0, 3);

	Vector expected = Vector_(3, 2.0/7.0, 10.0/7.0, -6.0/7.0);

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( matrix, CQP_example_automatic ) {

	Matrix A = Matrix_(3, 2,
			-1.0,  -1.0,
			-2.0,   1.0,
			 1.0,  -1.0);
	Matrix At = trans(A),
		   B = 2.0 * eye(3,3);

	Vector g = zero(3),
		   h = Vector_(2, 4.0, -2.0);

	Vector actState, actLam;
	boost::tie(actState, actLam) = solveCQP(B, A, g, h);

	Vector expected = Vector_(3, 2.0/7.0, 10.0/7.0, -6.0/7.0);

	CHECK(assert_equal(expected, actState));
	CHECK(actLam.size() == 2);
}

/* ************************************************************************* */

/** SQP example from SQP tutorial */
namespace sqp_example1 {

	/**
	 * objective function with gradient and hessian
	 * fx = (x2-2)^2 + x1^2;
	 */
	double objective(const Vector& x, boost::optional<Vector&> g = boost::none,
			  boost::optional<Matrix&> B = boost::none) {
		double x1 = x(0), x2 = x(1);
		if (g) *g = Vector_(2, 2.0*x1, 2.0*(x2-2.0));
		if (B) *B = 2.0 * eye(2,2);
		return (x2-2)*(x2-2) + x1*x1;
	}

	/**
	 * constraint function with gradient and hessian
	 * cx = 4*x1^2 + x2^2 - 1;
	 */
	Vector constraint(const Vector& x, boost::optional<Matrix&> A = boost::none,
			  boost::optional<Matrix&> B = boost::none) {
		double x1 = x(0), x2 = x(1);
		if (A) *A = Matrix_(2,1, 8.0*x1, 2.0*x2);
		if (B) *B = Matrix_(2,2,
				8.0, 0.0,
				0.0, 2.0);
		return Vector_(1, 4.0*x1*x1 + x2*x2 - 1.0);
	}

	/**
	 * evaluates a penalty function at a given point
	 */
	double penalty(const Vector& x) {
		double constraint_gain = 1.0;
		return objective(x) + constraint_gain*sum(abs(constraint(x)));
	}


}

/* ************************************************************************* */

/** SQP example from SQP tutorial (real saddle point) */
namespace sqp_example2 {

	/**
	 * objective function with gradient and hessian
	 * fx = (x2-2)^2 - x1^2;
	 */
	double objective(const Vector& x, boost::optional<Vector&> g = boost::none,
			  boost::optional<Matrix&> B = boost::none) {
		double x1 = x(0), x2 = x(1);
		if (g) *g = Vector_(2, -2.0*x1, 2.0*(x2-2.0));
		if (B) *B = Matrix_(2,2, -2.0, 0.0, 0.0, 2.0);
		return (x2-2)*(x2-2) - x1*x1;
	}

	/**
	 * constraint function with gradient and hessian
	 * cx = 4*x1^2 + x2^2 - 1;
	 */
	Vector constraint(const Vector& x, boost::optional<Matrix&> A = boost::none,
			  boost::optional<Matrix&> B = boost::none) {
		double x1 = x(0), x2 = x(1);
		if (A) *A = Matrix_(2,1, 8.0*x1, 2.0*x2);
		if (B) *B = Matrix_(2,2,
				8.0, 0.0,
				0.0, 2.0);
		return Vector_(1, 4.0*x1*x1 + x2*x2 - 1.0);
	}

	/**
	 * evaluates a penalty function at a given point
	 */
	double penalty(const Vector& x) {
		double constraint_gain = 1.0;
		return objective(x) + constraint_gain*sum(abs(constraint(x)));
	}
}

/* ************************************************************************* */
TEST( matrix, SQP_simple_analytic ) {
	using namespace sqp_example1;

	// parameters
	double stepsize = 0.25;
	size_t maxIt = 50;

	// initial conditions
	Vector x0 = Vector_(2, 2.0, 4.0),
		   lam0 = Vector_(1, -0.5);

	// current state
	Vector x = x0, lam = lam0;

	for (size_t i =0; i<maxIt; ++i) {

		// evaluate functions
		Vector dfx;
		Matrix dcx, ddfx, ddcx;
		objective(x, dfx, ddfx);
		Vector cx = constraint(x, dcx, ddcx);

		// use analytic hessian
		Matrix B = ddfx - lam(0)*ddcx;

		// solve subproblem
		Vector delta, lambda;
		boost::tie(delta, lambda) = solveCQP(B, -dcx, dfx, -cx);

		// update
		Vector step = stepsize * delta;
		x = x + step;
		lam = lambda;
	}

	// verify
	Vector expX = Vector_(2, 0.0, 1.0),
		   expLambda = Vector_(1, -1.0);

	CHECK(assert_equal(expX, x, 1e-4));
	CHECK(assert_equal(expLambda, lam, 1e-4));
}

/* ************************************************************************* */
TEST( matrix, SQP_simple_manual_bfgs ) {
	using namespace sqp_example1;

	// parameters
	double stepsize = 0.25;
	size_t maxIt = 50;

	// initial conditions
	Vector x0 = Vector_(2, 2.0, 4.0),
		   lam0 = Vector_(1, -0.5);

	// current state
	Vector x = x0, lam = lam0;
	Matrix Bi = eye(2,2);
	Vector step, prev_dfx;

	for (size_t i=0; i<maxIt; ++i) {

		// evaluate functions
		Vector dfx; Matrix dcx;
		objective(x, dfx);
		Vector cx = constraint(x, dcx);

		// Just use dfx for the Hessian
	    if (i>0) {
	        Vector Bis = Bi * step,
	        	   y = dfx - prev_dfx;
	        Bi = Bi + outer_prod(y, y) / inner_prod(y, step)
	        		- outer_prod(Bis, Bis) / inner_prod(step, Bis);
	    }
	    prev_dfx = dfx;

	    // solve subproblem
		Vector delta, lambda;
		boost::tie(delta, lambda) = solveCQP(Bi, -dcx, dfx, -cx);

		// update
		step = stepsize * delta;
		x = x + step;
		lam = lambda;
	}

	// verify
	Vector expX = Vector_(2, 0.0, 1.0),
		   expLambda = Vector_(1, -1.0);

	CHECK(assert_equal(expX, x, 1e-4));
	CHECK(assert_equal(expLambda, lam, 1e-4));
}

/* ************************************************************************* */
TEST( matrix, SQP_simple_bfgs1 ) {
	using namespace sqp_example1;

	// parameters
	size_t maxIt = 25;

	// initial conditions
	Vector x0 = Vector_(2, 2.0, 4.0),
		   lam0 = Vector_(1, -0.5);

	// create a BFGSEstimator
	BFGSEstimator hessian(2);

	// current state
	Vector x = x0, lam = lam0;
	Vector step;

	for (size_t i=0; i<maxIt; ++i) {

		// evaluate functions
		Vector dfx; Matrix dcx;
		objective(x, dfx);
		Vector cx = constraint(x, dcx);

		// Just use dfx for the Hessian
	    if (i>0) {
	    	hessian.update(dfx, step);
	    } else {
	    	hessian.update(dfx);
	    }

		// solve subproblem
		Vector delta, lambda;
		boost::tie(delta, lambda) = solveCQP(hessian.getB(), -dcx, dfx, -cx);
//		if (i == 0) print(delta, "delta1");

		// update
		step = linesearch(x,delta,penalty);
//		step = stepsize * delta;
		x = x + step;
		lam = lambda;
	}

	// verify
	Vector expX = Vector_(2, 0.0, 1.0),
		   expLambda = Vector_(1, -1.0);

	CHECK(assert_equal(expX, x, 1e-4));
	CHECK(assert_equal(expLambda, lam, 1e-4));
}

/* ************************************************************************* */
TEST( matrix, SQP_simple_bfgs2 ) {
	using namespace sqp_example2;

	// parameters
	double stepsize = 0.25;
	size_t maxIt = 50;

	// initial conditions
	Vector x0 = Vector_(2, 2.0, 4.0),
		   lam0 = Vector_(1, -0.5);

	// create a BFGSEstimator
	BFGSEstimator hessian(2);

	// current state
	Vector x = x0, lam = lam0;
	Vector step;

	for (size_t i=0; i<maxIt; ++i) {

		// evaluate functions
		Vector dfx; Matrix dcx;
		objective(x, dfx);
		Vector cx = constraint(x, dcx);

		// Just use dfx for the Hessian
	    if (i>0) {
	    	hessian.update(dfx, step);
	    } else {
	    	hessian.update(dfx);
	    }

		// solve subproblem
		Vector delta, lambda;
		boost::tie(delta, lambda) = solveCQP(hessian.getB(), -dcx, dfx, -cx);
//		if (i == 0) print(delta, "delta2");

		// update
//		step = linesearch(x,delta,penalty);
		step = stepsize * delta;
		x = x + step;
		lam = lambda;
	}

	// verify
	Vector expX = Vector_(2, 0.0, 1.0),
		   expLambda = Vector_(1, -1.0);

	// should determine the actual values for this one
//	CHECK(assert_equal(expX, x, 1e-4));
//	CHECK(assert_equal(expLambda, lam, 1e-4));
}

/* ************************************************************************* */
TEST( matrix, line_search ) {
	using namespace sqp_example2;

	// initial conditions
	Vector x0 = Vector_(2, 2.0, 4.0),
		   delta = Vector_(2, 0.85, -5.575);

	Vector actual = linesearch(x0,delta,penalty);

	// check that error goes down
	double init_error = penalty(x0),
		   final_error = penalty(x0 + actual);

	//double actual_stepsize = dot(actual, delta)/dot(delta, delta);
//	cout << "actual_stepsize: " << actual_stepsize << endl;

	CHECK(final_error <= init_error);
}

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
