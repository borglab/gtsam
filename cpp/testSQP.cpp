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
#include <NonlinearFactor.h>
#include <NonlinearEquality.h>
#include <NonlinearFactorGraph.h>
#include <NonlinearOptimizer.h>
#include <SQPOptimizer.h>
#include <Simulated2DMeasurement.h>
#include <simulated2D.h>
#include <Ordering.h>
#include <VSLAMConfig.h>
#include <VSLAMFactor.h>
#include <VSLAMGraph.h>
#include <SimpleCamera.h>

// templated implementations
#include <NonlinearFactorGraph-inl.h>
#include <NonlinearConstraint-inl.h>
#include <NonlinearOptimizer-inl.h>
#include <SQPOptimizer-inl.h>

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
	int maxIt = 10;
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
		VectorConfig delta = fg.optimize(ord).scale(-1.0);
		if (verbose) delta.print("Delta");

		// update initial estimate
		VectorConfig newState = state.exmap(delta);
		state = newState;

		if (verbose) state.print("Updated State");
	}

	// verify that it converges to the nearest optimal point
	VectorConfig expected;
	expected.insert("lam", Vector_(1, -1.0));
	expected.insert("x", Vector_(1, 2.12));
	expected.insert("y", Vector_(1, -0.5));
	CHECK(assert_equal(expected,state, 1e-2));
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
		VectorConfig newState = state.exmap(delta.scale(-1.0));

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
typedef boost::shared_ptr<NonlinearConstraint<VectorConfig> > shared_c;
typedef boost::shared_ptr<NonlinearEquality<VectorConfig> > shared_eq;
typedef boost::shared_ptr<VectorConfig> shared_cfg;
typedef NonlinearOptimizer<NLGraph,VectorConfig> Optimizer;
/**
 * Determining a ground truth linear system
 * with two poses seeing one landmark, with each pose
 * constrained to a particular value
 */
TEST (SQP, two_pose_truth ) {
	bool verbose = false;
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
	if (verbose) actual->print("Configuration after optimization");

	// verify
	VectorConfig expected(feas);
	expected.insert("l1", Vector_(2, 1.0, 6.0));
	CHECK(assert_equal(expected, *actual, 1e-5));
}

namespace sqp_test1 {
// binary constraint between landmarks
/** g(x) = x-y = 0 */
Vector g_func(const VectorConfig& config, const list<string>& keys) {
	return config[keys.front()]-config[keys.back()];
}

/** gradient at l1 */
Matrix grad_g1(const VectorConfig& config, const list<string>& keys) {
	return eye(2);
}

/** gradient at l2 */
Matrix grad_g2(const VectorConfig& config, const list<string>& keys) {
	return -1*eye(2);
}
} // \namespace sqp_test1

namespace sqp_test2 {
// Unary Constraint on x1
/** g(x) = x -[1;1] = 0 */
Vector g_func(const VectorConfig& config, const list<string>& keys) {
	return config[keys.front()]-Vector_(2, 1.0, 1.0);
}

/** gradient at x1 */
Matrix grad_g(const VectorConfig& config, const list<string>& keys) {
	return eye(2);
}
} // \namespace sqp_test2

/**
 *  Version that actually uses nonlinear equality constraints
 *  to to perform optimization.  Same as above, but no
 *  equality constraint on x2, and two landmarks that
 *  should be the same. Note that this is a linear system,
 *  so it will converge in one step.
 */
TEST (SQP, two_pose ) {
	bool verbose = false;
	// position (1, 1) constraint for x1
	VectorConfig feas;
	feas.insert("x1", Vector_(2, 1.0, 1.0));

	// constant constraint on x1
	boost::shared_ptr<NonlinearConstraint1<VectorConfig> > c1(
			new NonlinearConstraint1<VectorConfig>(
					"x1", *sqp_test2::grad_g,
					*sqp_test2::g_func, 2, "L_x1"));

	// measurement from x1 to l1
	Vector z1 = Vector_(2, 0.0, 5.0);
	double sigma1 = 0.1;
	shared f1(new Simulated2DMeasurement(z1, sigma1, "x1", "l1"));

	// measurement from x2 to l2
	Vector z2 = Vector_(2, -4.0, 0.0);
	double sigma2 = 0.1;
	shared f2(new Simulated2DMeasurement(z2, sigma2, "x2", "l2"));

	// equality constraint between l1 and l2
	boost::shared_ptr<NonlinearConstraint2<VectorConfig> > c2(
			new NonlinearConstraint2<VectorConfig>(
					"l1", *sqp_test1::grad_g1,
					"l2", *sqp_test1::grad_g2,
					*sqp_test1::g_func, 2, "L_l1l2"));

	// construct the graph
	NLGraph graph;
	graph.push_back(c1);
	graph.push_back(c2);
	graph.push_back(f1);
	graph.push_back(f2);

	// create an initial estimate
	shared_cfg initialEstimate(new VectorConfig(feas)); // must start with feasible set
	initialEstimate->insert("l1", Vector_(2, 1.0, 6.0)); // ground truth
	initialEstimate->insert("l2", Vector_(2, -4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert("x2", Vector_(2, 0.0, 0.0)); // other pose starts at origin

	// create an initial estimate for the lagrange multiplier
	shared_cfg initLagrange(new VectorConfig);
	initLagrange->insert("L_l1l2", Vector_(2, 1.0, 1.0));
	initLagrange->insert("L_x1", Vector_(2, 1.0, 1.0));

	// create state config variables and initialize them
	VectorConfig state(*initialEstimate), state_lam(*initLagrange);

	// optimization loop
	int maxIt = 1;
	for (int i = 0; i<maxIt; ++i) {
 		// linearize the graph
		GaussianFactorGraph fg;
		typedef FactorGraph<NonlinearFactor<VectorConfig> >::const_iterator const_iterator;
		typedef NonlinearConstraint<VectorConfig> NLConstraint;
		// iterate over all factors
		for (const_iterator factor = graph.begin(); factor < graph.end(); factor++) {
			const shared_c constraint = boost::shared_dynamic_cast<NLConstraint >(*factor);
			if (constraint == NULL) {
				// if a regular factor, linearize using the default linearization
				GaussianFactor::shared_ptr f = (*factor)->linearize(state);
				fg.push_back(f);
			} else {
				// if a constraint, linearize using the constraint method (2 configs)
				GaussianFactor::shared_ptr f, c;
				boost::tie(f,c) = constraint->linearize(state, state_lam);
				fg.push_back(f);
				fg.push_back(c);
			}
		}
		if (verbose) fg.print("Linearized graph");

		// create an ordering
		Ordering ordering;
		ordering += "x1", "x2", "l1", "l2", "L_l1l2", "L_x1";

		// optimize linear graph to get full delta config
		VectorConfig delta = fg.optimize(ordering);
		if (verbose) delta.print("Delta Config");

		// update both state variables
		state = state.exmap(delta);
		if (verbose) state.print("newState");
		state_lam = state_lam.exmap(delta);
		if (verbose) state_lam.print("newStateLam");
	}

	// verify
	VectorConfig expected(feas);
	expected.insert("l1", Vector_(2, 1.0, 6.0));
	expected.insert("l2", Vector_(2, 1.0, 6.0));
	expected.insert("x2", Vector_(2, 5.0, 6.0));
	CHECK(assert_equal(expected, state, 1e-5));
}

/* ********************************************************************* */
// VSLAM Examples
/* ********************************************************************* */
// make a realistic calibration matrix
double fov = 60; // degrees
size_t w=640,h=480;
Cal3_S2 K(fov,w,h);
boost::shared_ptr<Cal3_S2> shK(new Cal3_S2(K));

// typedefs for visual SLAM example
typedef boost::shared_ptr<VSLAMFactor> shared_vf;
typedef NonlinearOptimizer<VSLAMGraph,VSLAMConfig> VOptimizer;
typedef SQPOptimizer<VSLAMGraph, VSLAMConfig> SOptimizer;

/**
 * Ground truth for a visual slam example with stereo vision
 */
TEST (SQP, stereo_truth ) {
	bool verbose = false;

	// create initial estimates
	Rot3 faceDownY(Matrix_(3,3,
				1.0, 0.0, 0.0,
				0.0, 0.0, 1.0,
				0.0, 1.0, 0.0));
	Pose3 pose1(faceDownY, Point3()); // origin, left camera
	SimpleCamera camera1(K, pose1);
	Pose3 pose2(faceDownY, Point3(2.0, 0.0, 0.0)); // 2 units to the left
	SimpleCamera camera2(K, pose2);
	Point3 landmark(1.0, 5.0, 0.0); //centered between the cameras, 5 units away
	Point3 landmarkNoisy(1.0, 6.0, 0.0);

	// create truth config
	boost::shared_ptr<VSLAMConfig> truthConfig(new VSLAMConfig);
	truthConfig->addCameraPose(1, camera1.pose());
	truthConfig->addCameraPose(2, camera2.pose());
	truthConfig->addLandmarkPoint(1, landmark);

	// create graph
	VSLAMGraph graph;

	// create equality constraints for poses
	graph.addCameraConstraint(1, camera1.pose());
	graph.addCameraConstraint(2, camera2.pose());

	// create VSLAM factors
	Point2 z1 = camera1.project(landmark);
	if (verbose) z1.print("z1");
	shared_vf vf1(new VSLAMFactor(z1, 1.0, 1, 1, shK));
	graph.push_back(vf1);
	Point2 z2 = camera2.project(landmark);
	if (verbose) z2.print("z2");
	shared_vf vf2(new VSLAMFactor(z2, 1.0, 2, 1, shK));
	graph.push_back(vf2);

	if (verbose) graph.print("Graph after construction");

	// create ordering
	Ordering ord;
	ord += "x1", "x2", "l1";

	// create optimizer
	VOptimizer optimizer(graph, ord, truthConfig, 1e-5);

	// optimize
	VOptimizer afterOneIteration = optimizer.iterate();

	// verify
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

	// check if correct
	if (verbose) afterOneIteration.config()->print("After iteration");
	CHECK(assert_equal(*truthConfig,*(afterOneIteration.config())));
}


/**
 * Ground truth for a visual slam example with stereo vision
 * with some noise injected into the initial config
 */
TEST (SQP, stereo_truth_noisy ) {
	bool verbose = false;

	// setting to determine how far away the noisy landmark is,
	// given that the ground truth is 5m in front of the cameras
	double noisyDist = 7.6;

	// create initial estimates
	Rot3 faceDownY(Matrix_(3,3,
				1.0, 0.0, 0.0,
				0.0, 0.0, 1.0,
				0.0, 1.0, 0.0));
	Pose3 pose1(faceDownY, Point3()); // origin, left camera
	SimpleCamera camera1(K, pose1);
	Pose3 pose2(faceDownY, Point3(2.0, 0.0, 0.0)); // 2 units to the left
	SimpleCamera camera2(K, pose2);
	Point3 landmark(1.0, 5.0, 0.0); //centered between the cameras, 5 units away
	Point3 landmarkNoisy(1.0, noisyDist, 0.0); // initial point is too far out

	// create truth config
	boost::shared_ptr<VSLAMConfig> truthConfig(new VSLAMConfig);
	truthConfig->addCameraPose(1, camera1.pose());
	truthConfig->addCameraPose(2, camera2.pose());
	truthConfig->addLandmarkPoint(1, landmark);

	// create config
	boost::shared_ptr<VSLAMConfig> noisyConfig(new VSLAMConfig);
	noisyConfig->addCameraPose(1, camera1.pose());
	noisyConfig->addCameraPose(2, camera2.pose());
	noisyConfig->addLandmarkPoint(1, landmarkNoisy);

	// create graph
	VSLAMGraph graph;

	// create equality constraints for poses
	graph.addCameraConstraint(1, camera1.pose());
	graph.addCameraConstraint(2, camera2.pose());

	// create VSLAM factors
	Point2 z1 = camera1.project(landmark);
	if (verbose) z1.print("z1");
	shared_vf vf1(new VSLAMFactor(z1, 1.0, 1, 1, shK));
	graph.push_back(vf1);
	Point2 z2 = camera2.project(landmark);
	if (verbose) z2.print("z2");
	shared_vf vf2(new VSLAMFactor(z2, 1.0, 2, 1, shK));
	graph.push_back(vf2);

	if (verbose)  {
		graph.print("Graph after construction");
		noisyConfig->print("Initial config");
	}

	// create ordering
	Ordering ord;
	ord += "x1", "x2", "l1";

	// create optimizer
	VOptimizer optimizer(graph, ord, noisyConfig, 1e-5);

	if (verbose)
		cout << "Initial Error: " << optimizer.error() << endl;

	// use Levenberg-Marquardt optimization
	double relThresh = 1e-5, absThresh = 1e-5;
	optimizer = optimizer.levenbergMarquardt(relThresh, absThresh, VOptimizer::SILENT);

	// verify
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

	// check if correct
	if (verbose) {
		optimizer.config()->print("After iteration");
		cout << "Final error: " << optimizer.error() << endl;
	}
	CHECK(assert_equal(*truthConfig,*(optimizer.config())));
}

// Utility function to strip out a landmark number from a string key
int getNum(const string& key) {
	return atoi(key.substr(1, key.size()-1).c_str());
}

namespace sqp_stereo {
// binary constraint between landmarks
/** g(x) = x-y = 0 */
Vector g_func(const VSLAMConfig& config, const list<string>& keys) {
	return config.landmarkPoint(getNum(keys.front())).vector()-config.landmarkPoint(getNum(keys.back())).vector();
}

/** gradient at l1 */
Matrix grad_g1(const VSLAMConfig& config, const list<string>& keys) {
	return eye(3);
}

/** gradient at l2 */
Matrix grad_g2(const VSLAMConfig& config, const list<string>& keys) {
	return -1.0*eye(3);
}
} // \namespace sqp_test1

VSLAMGraph stereoExampleGraph() {
	// create initial estimates
	Rot3 faceDownY(Matrix_(3,3,
				1.0, 0.0, 0.0,
				0.0, 0.0, 1.0,
				0.0, 1.0, 0.0));
	Pose3 pose1(faceDownY, Point3()); // origin, left camera
	SimpleCamera camera1(K, pose1);
	Pose3 pose2(faceDownY, Point3(2.0, 0.0, 0.0)); // 2 units to the left
	SimpleCamera camera2(K, pose2);
	Point3 landmark1(1.0, 5.0, 0.0); //centered between the cameras, 5 units away
	Point3 landmark2(1.0, 5.0, 0.0);

	// create graph
	VSLAMGraph graph;

	// create equality constraints for poses
	graph.addCameraConstraint(1, camera1.pose());
	graph.addCameraConstraint(2, camera2.pose());

	// create VSLAM factors
	Point2 z1 = camera1.project(landmark1);
	shared_vf vf1(new VSLAMFactor(z1, 1.0, 1, 1, shK));
	graph.push_back(vf1);
	Point2 z2 = camera2.project(landmark2);
	shared_vf vf2(new VSLAMFactor(z2, 1.0, 2, 2, shK));
	graph.push_back(vf2);

	// create the binary equality constraint between the landmarks
	// NOTE: this is really just a linear constraint that is exactly the same
	// as the previous examples
	boost::shared_ptr<NonlinearConstraint2<VSLAMConfig> > c2(
				new NonlinearConstraint2<VSLAMConfig>(
						"l1", *sqp_stereo::grad_g1,
						"l2", *sqp_stereo::grad_g2,
						*sqp_stereo::g_func, 3, "L_l1l2"));
	graph.push_back(c2);

	return graph;
}

boost::shared_ptr<VSLAMConfig> stereoExampleTruthConfig() {
	// create initial estimates
	Rot3 faceDownY(Matrix_(3,3,
				1.0, 0.0, 0.0,
				0.0, 0.0, 1.0,
				0.0, 1.0, 0.0));
	Pose3 pose1(faceDownY, Point3()); // origin, left camera
	SimpleCamera camera1(K, pose1);
	Pose3 pose2(faceDownY, Point3(2.0, 0.0, 0.0)); // 2 units to the left
	SimpleCamera camera2(K, pose2);
	Point3 landmark1(1.0, 5.0, 0.0); //centered between the cameras, 5 units away
	Point3 landmark2(1.0, 5.0, 0.0);

	// create config
	boost::shared_ptr<VSLAMConfig> truthConfig(new VSLAMConfig);
	truthConfig->addCameraPose(1, camera1.pose());
	truthConfig->addCameraPose(2, camera2.pose());
	truthConfig->addLandmarkPoint(1, landmark1);
	truthConfig->addLandmarkPoint(2, landmark2); // create two landmarks in same place

	return truthConfig;
}

/**
 * SQP version of the above stereo example,
 * with the initial case as the ground truth
 */
TEST (SQP, stereo_sqp ) {
	bool verbose = false;

	// get a graph
	VSLAMGraph graph = stereoExampleGraph();
	if (verbose) graph.print("Graph after construction");

	// get the truth config
	boost::shared_ptr<VSLAMConfig> truthConfig = stereoExampleTruthConfig();

	// create ordering
	Ordering ord;
	ord += "x1", "x2", "l1", "l2";

	// create optimizer
	SOptimizer optimizer(graph, ord, truthConfig);

	// optimize
	SOptimizer afterOneIteration = optimizer.iterate();

	// check if correct
	CHECK(assert_equal(*truthConfig,*(afterOneIteration.config())));
}

/**
 * SQP version of the above stereo example,
 * with noise in the initial estimate
 */
TEST (SQP, stereo_sqp_noisy ) {
	bool verbose = false;

	// get a graph
	VSLAMGraph graph = stereoExampleGraph();

	// create initial data
	Rot3 faceDownY(Matrix_(3,3,
			1.0, 0.0, 0.0,
			0.0, 0.0, 1.0,
			0.0, 1.0, 0.0));
	Pose3 pose1(faceDownY, Point3()); // origin, left camera
	Pose3 pose2(faceDownY, Point3(2.0, 0.0, 0.0)); // 2 units to the left
	Point3 landmark1(0.5, 5.0, 0.0); //centered between the cameras, 5 units away
	Point3 landmark2(1.5, 5.0, 0.0);

	// noisy config
	boost::shared_ptr<VSLAMConfig> initConfig(new VSLAMConfig);
	initConfig->addCameraPose(1, pose1);
	initConfig->addCameraPose(2, pose2);
	initConfig->addLandmarkPoint(1, landmark1);
	initConfig->addLandmarkPoint(2, landmark2); // create two landmarks in same place

	// create ordering
	Ordering ord;
	ord += "x1", "x2", "l1", "l2";

	// create optimizer
	SOptimizer optimizer(graph, ord, initConfig);

	// optimize
	double start_error = optimizer.error();
	int maxIt = 2;
	for (int i=0; i<maxIt; ++i) {
		if (verbose) cout << "\n ************************** \n"
						  << " Iteration: " << i << endl;
		//if (verbose) optimizer.graph()->print();
		if (verbose) optimizer.config()->print();
		if (verbose)
			optimizer = optimizer.iterate(SOptimizer::FULL);
		else
			optimizer = optimizer.iterate(SOptimizer::SILENT);
	}

	if (verbose) cout << "Initial Error: " << start_error << "\n"
					  << "Final Error:   " << optimizer.error() << endl;

	// get the truth config
	boost::shared_ptr<VSLAMConfig> truthConfig = stereoExampleTruthConfig();

	if (verbose) {
		initConfig->print("Initial Config");
		truthConfig->print("Truth Config");
		optimizer.config()->print("After optimization");
	}

	// check if correct
	CHECK(assert_equal(*truthConfig,*(optimizer.config())));
}

/**
 * SQP version of the above stereo example,
 * with noise in the initial estimate and manually specified
 * lagrange multipliers
 */
TEST (SQP, stereo_sqp_noisy_manualLagrange ) {
	bool verbose = false;

	// get a graph
	VSLAMGraph graph = stereoExampleGraph();

	// create initial data
	Rot3 faceDownY(Matrix_(3,3,
			1.0, 0.0, 0.0,
			0.0, 0.0, 1.0,
			0.0, 1.0, 0.0));
	Pose3 pose1(faceDownY, Point3()); // origin, left camera
	Pose3 pose2(faceDownY, Point3(2.0, 0.0, 0.0)); // 2 units to the left
	Point3 landmark1(0.5, 5.0, 0.0); //centered between the cameras, 5 units away
	Point3 landmark2(1.5, 5.0, 0.0);

	// noisy config
	boost::shared_ptr<VSLAMConfig> initConfig(new VSLAMConfig);
	initConfig->addCameraPose(1, pose1);
	initConfig->addCameraPose(2, pose2);
	initConfig->addLandmarkPoint(1, landmark1);
	initConfig->addLandmarkPoint(2, landmark2); // create two landmarks in same place

	// create ordering with lagrange multiplier included
	Ordering ord;
	ord += "x1", "x2", "l1", "l2", "L_l1l2";

	// create lagrange multipliers
	SOptimizer::shared_vconfig initLagrangeConfig(new VectorConfig);
	initLagrangeConfig->insert("L_l1l2", Vector_(3, 0.0, 0.0, 0.0));

	// create optimizer
	SOptimizer optimizer(graph, ord, initConfig, initLagrangeConfig);

	// optimize
	double start_error = optimizer.error();
	int maxIt = 5;
	for (int i=0; i<maxIt; ++i) {
		if (verbose) {
			cout << "\n ************************** \n"
				 << " Iteration: " << i << endl;
			optimizer.config()->print("Config Before Iteration");
			optimizer.configLagrange()->print("Lagrange Before Iteration");
			optimizer = optimizer.iterate(SOptimizer::FULL);
		}
		else
			optimizer = optimizer.iterate(SOptimizer::SILENT);
	}

	if (verbose) cout << "Initial Error: " << start_error << "\n"
					  << "Final Error:   " << optimizer.error() << endl;

	// get the truth config
	boost::shared_ptr<VSLAMConfig> truthConfig = stereoExampleTruthConfig();

	if (verbose) {
		initConfig->print("Initial Config");
		truthConfig->print("Truth Config");
		optimizer.config()->print("After optimization");
	}

	// check if correct
	CHECK(assert_equal(*truthConfig,*(optimizer.config())));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
