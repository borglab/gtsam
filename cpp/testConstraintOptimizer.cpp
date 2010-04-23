/**
 * @file testConstraintOptimizer.cpp
 * @brief Tests the optimization engine for SQP and BFGS Quadratic programming techniques
 * @author Alex Cunningham
 */

#include <iostream>

#include <boost/tuple/tuple.hpp>
#include <boost/optional.hpp>

#include <CppUnitLite/TestHarness.h>

#include <ConstraintOptimizer.h>

using namespace std;
using namespace gtsam;

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
	Vector objective(const Vector& x, boost::optional<Vector&> g = boost::none,
			  boost::optional<Matrix&> B = boost::none) {
		double x1 = x(0), x2 = x(1);
		if (g) *g = Vector_(2, 2.0*x1, 2.0*(x2-2.0));
		if (B) *B = 2.0 * eye(2,2);
		return Vector_(1, (x2-2)*(x2-2) + x1*x1);
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
		Vector fx = objective(x, dfx, ddfx);
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
		Vector fx = objective(x, dfx);
		Vector cx = constraint(x, dcx);

		// Just use dfx for the Hessian
	    if (i>0) {
	        Vector Bis = Bi * step,
	        	   y = dfx - prev_dfx;
	        Bi = Bi + outer_prod(y, y) / inner_prod(y, step)
	        		- outer_prod(Bis, Bis) / inner_prod(step, Bis);
	    }
	    prev_dfx = dfx;

	    // use bfgs
		Matrix B = Bi;

		// solve subproblem
		Vector delta, lambda;
		boost::tie(delta, lambda) = solveCQP(B, -dcx, dfx, -cx);

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
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
