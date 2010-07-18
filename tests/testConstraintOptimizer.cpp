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

SharedDiagonal probModel1 = sharedSigma(1,1.0);
SharedDiagonal probModel2 = sharedSigma(2,1.0);
SharedDiagonal constraintModel1 = noiseModel::Constrained::All(1);

/* *********************************************************************
 * This example uses a nonlinear objective function and
 * nonlinear equality constraint.  The formulation is actually
 * the Cholesky form that creates the full Hessian explicitly,
 * which should really be avoided with our QR-based machinery.
 *
 * Note: the update equation used here has a fixed step size
 * and gain that is rather arbitrarily chosen, and as such,
 * will take a silly number of iterations.
 */
TEST (SQP, problem1_cholesky ) {
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

/* *********************************************************************
 * This example uses a nonlinear objective function and
 * nonlinear equality constraint.  This formulation splits
 * the constraint into a factor and a linear constraint.
 *
 * This example uses the same silly number of iterations as the
 * previous example.
 */
TEST (SQP, problem1_sqp ) {
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
	int maxIt = 5;
	for (int i = 0; i<maxIt; ++i) {
		if (verbose) cout << "\n******************************\nIteration: " << i+1 << endl;

		// extract the states
		double x, y, lambda;
		x = state[x1](0);
		y = state[y1](0);
		lambda = state[L1](0);

		/** create the linear factor
		 * ||h(x)-z||^2 => ||Ax-b||^2
		 *  where:
		 *		h(x) simply returns the inputs
		 *		z    zeros(2)
		 *		A 	 identity
		 *		b	 linearization point
		 */
		Matrix A = eye(2);
		Vector b = Vector_(2, x, y);
		GaussianFactor::shared_ptr f1(
						new GaussianFactor(x1, sub(A, 0,2, 0,1), // A(:,1)
										   y1, sub(A, 0,2, 1,2), // A(:,2)
										   b,                     // rhs of f(x)
										   probModel2));          // arbitrary sigma

		/** create the constraint-linear factor
		 * Provides a mechanism to use variable gain to force the constraint
		 * \lambda*gradG*dx + d\lambda = zero
		 * formulated in matrix form as:
		 * [\lambda*gradG eye(1)] [dx; d\lambda] = zero
		 */
		Matrix gradG = Matrix_(1, 2,2*x, -1.0);
		GaussianFactor::shared_ptr f2(
				new GaussianFactor(x1, lambda*sub(gradG, 0,1, 0,1), // scaled gradG(:,1)
								   y1, lambda*sub(gradG, 0,1, 1,2), // scaled gradG(:,2)
								   L1, eye(1),                      // dlambda term
								   Vector_(1, 0.0),                  // rhs is zero
								   probModel1));                     // arbitrary sigma

		// create the actual constraint
		// [gradG] [x; y] - g = 0
		Vector g = Vector_(1,x*x-y-5);
		GaussianFactor::shared_ptr c1(
				new GaussianFactor(x1, sub(gradG, 0,1, 0,1),   // slice first part of gradG
								   y1, sub(gradG, 0,1, 1,2),   // slice second part of gradG
								   g,                           // value of constraint function
								   constraintModel1));          // force to constraint

		// construct graph
		GaussianFactorGraph fg;
		fg.push_back(f1);
		fg.push_back(f2);
		fg.push_back(c1);
		if (verbose) fg.print("Graph");

		// solve
		Ordering ord;
		ord += x1, y1, L1;
		VectorConfig delta = fg.optimize(ord);
		if (verbose) delta.print("Delta");

		// update initial estimate
		VectorConfig newState = expmap(state, delta.scale(-1.0));

		// set the state to the updated state
		state = newState;

		if (verbose) state.print("Updated State");
	}

	// verify that it converges to the nearest optimal point
	VectorConfig expected;
	expected.insert(x1, Vector_(1, 2.12));
	expected.insert(y1, Vector_(1, -0.5));
	CHECK(assert_equal(state[x1], expected[x1], 1e-2));
	CHECK(assert_equal(state[y1], expected[y1], 1e-2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
