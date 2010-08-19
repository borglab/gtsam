/**
 * @file testConstraintOptimizer.cpp
 * @brief Tests the optimization engine for SQP and BFGS Quadratic programming techniques
 * @author Alex Cunningham
 */

/** IMPORTANT NOTE: this file is only compiled when LDL is available */

#include <iostream>
#include <limits>

#include <boost/tuple/tuple.hpp>
#include <boost/optional.hpp>

#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/inference/Ordering.h>
#include <gtsam/nonlinear/ConstraintOptimizer.h>

#define GTSAM_MAGIC_KEY

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;

/* *********************************************************************
 * Example from SQP testing:
 *
 * This example uses a nonlinear objective function and
 * nonlinear equality constraint.  The formulation is actually
 * the Cholesky form that creates the full Hessian explicitly,
 * and isn't expecially compatible with our machinery.
 */
TEST (NonlinearConstraint, problem1_cholesky ) {
	bool verbose = false;
	// use a nonlinear function of f(x) = x^2+y^2
	// nonlinear equality constraint: g(x) = x^2-5-y=0
	// Lagrangian: f(x) + \lambda*g(x)

	// Symbols
	Symbol x1("x1"), y1("y1"), L1("L1");

	// state structure: [x y \lambda]
	VectorConfig init, state;
	init.insert(x1, Vector_(1, 1.0));
	init.insert(y1, Vector_(1, 1.0));
	init.insert(L1, Vector_(1, 1.0));
	state = init;

	if (verbose) init.print("Initial State");

	// loop until convergence
	int maxIt = 10;
	for (int i = 0; i<maxIt; ++i) {
		if (verbose) cout << "\n******************************\nIteration: " << i+1 << endl;

		// extract the states
		double x, y, lambda;
		x = state[x1](0);
		y = state[y1](0);
		lambda = state[L1](0);

		// calculate the components
		Matrix H1, H2, gradG;
		Vector gradL, gx;

		// hessian of lagrangian function, in two columns:
		H1 = Matrix_(2,1,
				2.0+2.0*lambda,
				0.0);
		H2 = Matrix_(2,1,
				0.0,
				2.0);

		// deriviative of lagrangian function
		gradL = Vector_(2,
				2.0*x*(1+lambda),
				2.0*y-lambda);

		// constraint derivatives
		gradG = Matrix_(2,1,
				2.0*x,
				0.0);

		// constraint value
		gx = Vector_(1,
				x*x-5-y);

		// create a factor for the states
		GaussianFactor::shared_ptr f1(new
				GaussianFactor(x1, H1, y1, H2, L1, gradG, gradL, probModel2));

		// create a factor for the lagrange multiplier
		GaussianFactor::shared_ptr f2(new
				GaussianFactor(x1, -sub(gradG, 0, 1, 0, 1),
							   y1, -sub(gradG, 1, 2, 0, 1), -gx, constraintModel1));

		// construct graph
		GaussianFactorGraph fg;
		fg.push_back(f1);
		fg.push_back(f2);
		if (verbose) fg.print("Graph");

		// solve
		Ordering ord;
		ord += x1, y1, L1;
		VectorConfig delta = fg.optimize(ord).scale(-1.0);
		if (verbose) delta.print("Delta");

		// update initial estimate
		VectorConfig newState = expmap(state, delta);
		state = newState;

		if (verbose) state.print("Updated State");
	}

	// verify that it converges to the nearest optimal point
	VectorConfig expected;
	expected.insert(L1, Vector_(1, -1.0));
	expected.insert(x1, Vector_(1, 2.12));
	expected.insert(y1, Vector_(1, -0.5));
	CHECK(assert_equal(expected,state, 1e-2));
}


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
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
