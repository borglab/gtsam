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
#include <NonlinearEquality.h>
#include <NonlinearFactorGraph.h>
#include <NonlinearOptimizer.h>
#include <Simulated2DMeasurement.h>
#include <simulated2D.h>
#include <Ordering.h>

// templated implementations
#include <NonlinearFactorGraph-inl.h>
#include "NonlinearOptimizer-inl.h"

using namespace std;
using namespace gtsam;
using namespace boost::assign;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

/**
 * This example uses a nonlinear objective function and
 * nonlinear equality constraint.  The formulation is actually
 * the Choleski form that creates the full Hessian explicitly,
 * which should really be avoided with our QR-based machinery.
 *
 * Note: the update equation used here has a fixed step size
 * and gain that is rather arbitrarily chosen, and as such,
 * will take a silly number of iterations.
 */
TEST (SQP, problem1_choleski ) {
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

/**
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
	// Lagrangian: f(x) + lam*g(x)

	// state structure: [x y lam]
	VectorConfig init, state;
	init.insert("x", Vector_(1, 1.0));
	init.insert("y", Vector_(1, 1.0));
	init.insert("lam", Vector_(1, 1.0));
	state = init;

	if (verbose) init.print("Initial State");

	// loop until convergence
	int maxIt = 5;
	for (int i = 0; i<maxIt; ++i) {
		if (verbose) cout << "\n******************************\nIteration: " << i+1 << endl;

		// extract the states
		double x, y, lam;
		x = state["x"](0);
		y = state["y"](0);
		lam = state["lam"](0);

		// create components
		Matrix A = eye(2);
		Matrix gradG = Matrix_(1, 2,
				2*x, -1.0);
		Vector g = Vector_(1,
				x*x-y-5);
		Vector b = Vector_(2, x, y);

		/** create the linear factor
		 * ||h(x)-z||^2 => ||Ax-b||^2
		 *  where:
		 *		h(x) simply returns the inputs
		 *		z    zeros(2)
		 *		A 	 identity
		 *		b	 linearization point
		 */
		GaussianFactor::shared_ptr f1(
						new GaussianFactor("x", sub(A, 0,2, 0,1), // A(:,1)
										   "y", sub(A, 0,2, 1,2), // A(:,2)
										   b,                     // rhs of f(x)
										   1.0));                 // arbitrary sigma

		/** create the constraint-linear factor
		 * Provides a mechanism to use variable gain to force the constraint
		 * to zero
		 * lam*gradG*dx + dlam + lam
		 * formulated in matrix form as:
		 * [lam*gradG eye(1)] [dx; dlam] = zero
		 */
		GaussianFactor::shared_ptr f2(
				new GaussianFactor("x", lam*sub(gradG, 0,1, 0,1), // scaled gradG(:,1)
								   "y", lam*sub(gradG, 0,1, 1,2), // scaled gradG(:,2)
								   "lam", eye(1),     // dlam term
								   Vector_(1, 0.0),             // rhs is zero
								   1.0));                         // arbitrary sigma

		// create the actual constraint
		// [gradG] [x; y]- g = 0
		GaussianFactor::shared_ptr c1(
				new GaussianFactor("x", sub(gradG, 0,1, 0,1),   // slice first part of gradG
								   "y", sub(gradG, 0,1, 1,2),   // slice second part of gradG
								   g,                           // value of constraint function
								   0.0));                       // force to constraint

		// construct graph
		GaussianFactorGraph fg;
		fg.push_back(f1);
		fg.push_back(f2);
		fg.push_back(c1);
		if (verbose) fg.print("Graph");

		// solve
		Ordering ord;
		ord += "x", "y", "lam";
		VectorConfig delta = fg.optimize(ord);
		if (verbose) delta.print("Delta");

		// update initial estimate
		double gain = 1.0;
		// Note:
		// It appears that exmap uses adding rather than subtracting to
		// update, which breaks the system.
		//VectorConfig newState = state.exmap(delta);
		VectorConfig newState;
		newState.insert("x", state["x"]-gain*delta["x"]);
		newState.insert("y", state["y"]-gain*delta["y"]);
		newState.insert("lam", state["lam"]-gain*delta["lam"]);

		// set the state to the updated state
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

// components for nonlinear factor graphs
bool vector_compare(const std::string& key,
					const VectorConfig& feasible,
					const VectorConfig& input) {
	Vector feas, lin;
	feas = feasible[key];
	lin = input[key];
	return equal_with_abs_tol(lin, feas, 1e-5);
}
typedef NonlinearFactorGraph<VectorConfig> NLGraph;
typedef boost::shared_ptr<NonlinearFactor<VectorConfig> > shared;
typedef boost::shared_ptr<NonlinearEquality<VectorConfig> > shared_eq;
typedef NonlinearOptimizer<NLGraph,VectorConfig> Optimizer;
/**
 * Determining a ground truth nonlinear system
 * with two poses seeing one landmark, with each pose
 * constrained to a particular value
 */
TEST (SQP, two_pose_truth ) {
	// position (1, 1) constraint for x1
	// position (5, 6) constraint for x2
	VectorConfig feas;
	feas.insert("x1", Vector_(2, 1.0, 1.0));
	feas.insert("x2", Vector_(2, 5.0, 6.0));

	// constant constraint on x1
	shared_eq ef1(new NonlinearEquality<VectorConfig>("x1", feas, 2, *vector_compare));

	// constant constraint on x2
	shared_eq ef2(new NonlinearEquality<VectorConfig>("x2", feas, 2, *vector_compare));

	// measurement from x1 to l1
	Vector z1 = Vector_(2, 0.0, 5.0);
	double sigma1 = 0.1;
	shared f1(new Simulated2DMeasurement(z1, sigma1, "x1", "l1"));

	// measurement from x2 to l1
	Vector z2 = Vector_(2, -4.0, 0.0);
	double sigma2 = 0.1;
	shared f2(new Simulated2DMeasurement(z2, sigma2, "x2", "l1"));

	// construct the graph
	NLGraph graph;
	graph.push_back(ef1);
	graph.push_back(ef2);
	graph.push_back(f1);
	graph.push_back(f2);

	// create an initial estimate
	boost::shared_ptr<VectorConfig> initialEstimate(new VectorConfig(feas)); // must start with feasible set
	initialEstimate->insert("l1", Vector_(2, 1.0, 6.0)); // ground truth
	//initialEstimate->insert("l1", Vector_(2, 1.2, 5.6)); // with small error

	// optimize the graph
	Ordering ordering;
	ordering += "x1", "x2", "l1";
	Optimizer optimizer(graph, ordering, initialEstimate, 1e-5);

	// display solution
	double relativeThreshold = 1e-5;
	double absoluteThreshold = 1e-5;
	Optimizer act_opt = optimizer.gaussNewton(relativeThreshold, absoluteThreshold);
	boost::shared_ptr<const VectorConfig> actual = act_opt.config();
	//actual->print("Configuration after optimization");

	// verify
	VectorConfig expected(feas);
	expected.insert("l1", Vector_(2, 1.0, 6.0));
	CHECK(assert_equal(expected, *actual, 1e-5));
}

// Test of binary nonlinear equality constraint disabled until nonlinear constraints work

///**
// *  Version that actually uses nonlinear equality constraints
// *  to to perform optimization.  Same as above, but no
// *  equality constraint on x2, and two landmarks that
// *  should be the same.
// */
//TEST (SQP, two_pose ) {
//	// position (1, 1) constraint for x1
//	VectorConfig feas;
//	feas.insert("x1", Vector_(2, 1.0, 1.0));
//
//	// constant constraint on x1
//	shared_eq ef1(new NonlinearEquality<VectorConfig>("x1", feas, 2, *vector_compare));
//
//	// measurement from x1 to l1
//	Vector z1 = Vector_(2, 0.0, 5.0);
//	double sigma1 = 0.1;
//	shared f1(new Simulated2DMeasurement(z1, sigma1, "x1", "l1"));
//
//	// measurement from x2 to l2
//	Vector z2 = Vector_(2, -4.0, 0.0);
//	double sigma2 = 0.1;
//	shared f2(new Simulated2DMeasurement(z2, sigma2, "x2", "l1"));
//
//	// equality constraint between l1 and l2
//	boost::shared_ptr<NonlinearConstraint<VectorConfig> > c1(
//			new NonlinearConstraint<VectorConfig>(
//					"l1", "l2",        // specify nodes constrained
//					*g_function,       // evaluate the cost
//					*gradG_function)); // construct the gradient of g(x)
//
//	// construct the graph
//	NLGraph graph;
//	graph.push_back(ef1);
//	graph.push_back(c1);
//	graph.push_back(f1);
//	graph.push_back(f2);
//
//	// create an initial estimate
//	boost::shared_ptr<VectorConfig> initialEstimate(new VectorConfig(feas)); // must start with feasible set
//	initialEstimate->insert("l1", Vector_(2, 1.0, 6.0)); // ground truth
//	//initialEstimate->insert("l1", Vector_(2, 1.2, 5.6)); // with small error
//
//	// optimize the graph
//	Ordering ordering;
//	ordering += "x1", "x2", "l1";
//	Optimizer optimizer(graph, ordering, initialEstimate, 1e-5);
//
//	// display solution
//	double relativeThreshold = 1e-5;
//	double absoluteThreshold = 1e-5;
//	Optimizer act_opt = optimizer.gaussNewton(relativeThreshold, absoluteThreshold);
//	boost::shared_ptr<const VectorConfig> actual = act_opt.config();
//	//actual->print("Configuration after optimization");
//
//	// verify
//	VectorConfig expected(feas);
//	expected.insert("l1", Vector_(2, 1.0, 6.0));
//	CHECK(assert_equal(expected, *actual, 1e-5));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
