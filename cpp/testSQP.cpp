/*
 * @file testSQP.cpp
 * @brief demos of SQP using existing gtsam components
 * @author Alex Cunningham
 */

#include <iostream>
#include <cmath>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/map.hpp> // for insert
#include <boost/foreach.hpp>
#include <CppUnitLite/TestHarness.h>
#include <GaussianFactorGraph.h>
#include <Ordering.h>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

/**
 * This example uses a nonlinear objective function and
 * nonlinear equality constraint.  The formulation is actually
 * the Choleski form that creates the full Hessian explicitly,
 * which should really be avoided with our QR-based machinery
 */
TEST (SQP, problem1 ) {
	bool verbose = false;
	// use a nonlinear function of f(x) = x^2+y^2
	// nonlinear equality constraint: g(x) = x^2-5-y=0
	// Lagrangian: f(x) + lam*g(x)

	// state structure: [x y lam]
	VectorConfig init, state;
	init.insert("x", Vector_(1, 1.0));
	init.insert("y", Vector_(1, 1.0));
	init.insert("lam", Vector_(1, 1.0));
	state = init;

	if (verbose) init.print("Initial State");

	// loop until convergence
	int maxIt = 50;
	for (int i = 0; i<maxIt; ++i) {
		if (verbose) cout << "\n******************************\nIteration: " << i+1 << endl;

		// extract the states
		double x, y, lam;
		x = state["x"](0);
		y = state["y"](0);
		lam = state["lam"](0);

		// calculate the components
		Matrix H1, H2, gradG;
		Vector gradL, gx;

		// hessian of lagrangian function, in two columns:
		H1 = Matrix_(2,1,
				2.0+2.0*lam,
				0.0);
		H2 = Matrix_(2,1,
				0.0,
				2.0);

		// deriviative of lagrangian function
		gradL = Vector_(2,
				2.0*x*(1+lam),
				2.0*y-lam);

		// constraint derivatives
		gradG = Matrix_(2,1,
				2.0*x,
				0.0);

		// constraint value
		gx = Vector_(1,
				x*x-5-y);

		// create a factor for the states
		GaussianFactor::shared_ptr f1(new
				GaussianFactor("x", H1, "y", H2, "lam", gradG, gradL, 1.0));

		// create a factor for the lagrange multiplier
		GaussianFactor::shared_ptr f2(new
				GaussianFactor("x", -sub(gradG, 0, 1, 0, 1),
							   "y", -sub(gradG, 1, 2, 0, 1), -gx, 0.0));

		// construct graph
		GaussianFactorGraph fg;
		fg.push_back(f1);
		fg.push_back(f2);
		if (verbose) fg.print("Graph");

		// solve
		Ordering ord;
		ord += "x", "y", "lam";
		VectorConfig delta = fg.optimize(ord);
		if (verbose) delta.print("Delta");

		// update initial estimate
		double gain = 0.3;
		VectorConfig newState;
		newState.insert("x", state["x"]-gain*delta["x"]);
		newState.insert("y", state["y"]-gain*delta["y"]);
		newState.insert("lam", state["lam"]-gain*delta["lam"]);
		state = newState;

		if (verbose) state.print("Updated State");
	}

	// verify that it converges to the nearest optimal point
	VectorConfig expected;
	expected.insert("x", Vector_(1, 2.12));
	expected.insert("y", Vector_(1, -0.5));
	CHECK(assert_equal(state["x"], expected["x"], 1e-2));
	CHECK(assert_equal(state["y"], expected["y"], 1e-2));

}
	

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
