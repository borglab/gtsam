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
#include <boost/shared_ptr.hpp>
#include <CppUnitLite/TestHarness.h>

// TODO: DANGEROUS, create shared pointers
#define GTSAM_MAGIC_GAUSSIAN 2
#define GTSAM_MAGIC_KEY

#include <Pose3.h>
#include <GaussianFactorGraph.h>
#include <NonlinearOptimizer.h>
#include <SQPOptimizer.h>
#include <simulated2D.h>
#include <Ordering.h>
#include <visualSLAM.h>

// templated implementations
#include <NonlinearFactorGraph-inl.h>
#include <NonlinearConstraint-inl.h>
#include <NonlinearOptimizer-inl.h>
#include <SQPOptimizer-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost;
using namespace boost::assign;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

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

	// state structure: [x y \lambda]
	VectorConfig init, state;
	init.insert("x", Vector_(1, 1.0));
	init.insert("y", Vector_(1, 1.0));
	init.insert("L", Vector_(1, 1.0));
	state = init;

	if (verbose) init.print("Initial State");

	// loop until convergence
	int maxIt = 10;
	for (int i = 0; i<maxIt; ++i) {
		if (verbose) cout << "\n******************************\nIteration: " << i+1 << endl;

		// extract the states
		double x, y, lambda;
		x = state["x"](0);
		y = state["y"](0);
		lambda = state["L"](0);

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
				GaussianFactor("x", H1, "y", H2, "L", gradG, gradL, 1.0));

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
		ord += "x", "y", "L";
		VectorConfig delta = fg.optimize(ord).scale(-1.0);
		if (verbose) delta.print("Delta");

		// update initial estimate
		VectorConfig newState = expmap(state, delta);
		state = newState;

		if (verbose) state.print("Updated State");
	}

	// verify that it converges to the nearest optimal point
	VectorConfig expected;
	expected.insert("L", Vector_(1, -1.0));
	expected.insert("x", Vector_(1, 2.12));
	expected.insert("y", Vector_(1, -0.5));
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

	// state structure: [x y \lambda]
	VectorConfig init, state;
	init.insert("x", Vector_(1, 1.0));
	init.insert("y", Vector_(1, 1.0));
	init.insert("L", Vector_(1, 1.0));
	state = init;

	if (verbose) init.print("Initial State");

	// loop until convergence
	int maxIt = 5;
	for (int i = 0; i<maxIt; ++i) {
		if (verbose) cout << "\n******************************\nIteration: " << i+1 << endl;

		// extract the states
		double x, y, lambda;
		x = state["x"](0);
		y = state["y"](0);
		lambda = state["L"](0);

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
						new GaussianFactor("x", sub(A, 0,2, 0,1), // A(:,1)
										   "y", sub(A, 0,2, 1,2), // A(:,2)
										   b,                     // rhs of f(x)
										   1.0));                 // arbitrary sigma

		/** create the constraint-linear factor
		 * Provides a mechanism to use variable gain to force the constraint
		 * \lambda*gradG*dx + d\lambda = zero
		 * formulated in matrix form as:
		 * [\lambda*gradG eye(1)] [dx; d\lambda] = zero
		 */
		Matrix gradG = Matrix_(1, 2,2*x, -1.0);
		GaussianFactor::shared_ptr f2(
				new GaussianFactor("x", lambda*sub(gradG, 0,1, 0,1), // scaled gradG(:,1)
								   "y", lambda*sub(gradG, 0,1, 1,2),         // scaled gradG(:,2)
								   "L", eye(1),                         // dlambda term
								   Vector_(1, 0.0),                          // rhs is zero
								   1.0));                                    // arbitrary sigma

		// create the actual constraint
		// [gradG] [x; y] - g = 0
		Vector g = Vector_(1,x*x-y-5);
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
		ord += "x", "y", "L";
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
	expected.insert("x", Vector_(1, 2.12));
	expected.insert("y", Vector_(1, -0.5));
	CHECK(assert_equal(state["x"], expected["x"], 1e-2));
	CHECK(assert_equal(state["y"], expected["y"], 1e-2));
}

/* ********************************************************************* */

typedef simulated2D::Config Config2D;
typedef NonlinearFactorGraph<Config2D> NLGraph;
typedef NonlinearEquality<Config2D, simulated2D::PoseKey, Point2> NLE;
typedef boost::shared_ptr<simulated2D::Measurement > shared;
typedef NonlinearOptimizer<NLGraph, Config2D> Optimizer;

typedef TypedSymbol<Vector, 'L'> LamKey;

/*
 * Determining a ground truth linear system
 * with two poses seeing one landmark, with each pose
 * constrained to a particular value
 */
TEST (SQP, two_pose_truth ) {
	bool verbose = false;

	// create a graph
	shared_ptr<NLGraph> graph(new NLGraph);

	// add the constraints on the ends
	// position (1, 1) constraint for x1
	// position (5, 6) constraint for x2
	simulated2D::PoseKey x1(1), x2(2);
	simulated2D::PointKey l1(1);
	Point2 pt_x1(1.0, 1.0),
		   pt_x2(5.0, 6.0);
	shared_ptr<NLE> ef1(new NLE(x1, pt_x1)),
			        ef2(new NLE(x2, pt_x2));
	graph->push_back(ef1);
	graph->push_back(ef2);

	// measurement from x1 to l1
	Point2 z1(0.0, 5.0);
	sharedGaussian sigma(noiseModel::Isotropic::Sigma(2, 0.1));
	shared f1(new simulated2D::Measurement(z1, sigma, x1,l1));
	graph->push_back(f1);

	// measurement from x2 to l1
	Point2 z2(-4.0, 0.0);
	shared f2(new simulated2D::Measurement(z2, sigma, x2,l1));
	graph->push_back(f2);

	// create an initial estimate
	Point2 pt_l1(
			1.0, 6.0 // ground truth
		  //1.2, 5.6 // small error
			);
	shared_ptr<Config2D> initialEstimate(new Config2D);
	initialEstimate->insert(l1, pt_l1);
	initialEstimate->insert(x1, pt_x1);
	initialEstimate->insert(x2, pt_x2);

	// optimize the graph
	shared_ptr<Ordering> ordering(new Ordering());
	*ordering += "x1", "x2", "l1";
	Optimizer::shared_solver solver(new Optimizer::solver(ordering));
	Optimizer optimizer(graph, initialEstimate, solver);

	// display solution
	double relativeThreshold = 1e-5;
	double absoluteThreshold = 1e-5;
	Optimizer act_opt = optimizer.gaussNewton(relativeThreshold, absoluteThreshold);
	boost::shared_ptr<const Config2D> actual = act_opt.config();
	if (verbose) actual->print("Configuration after optimization");

	// verify
	Config2D expected;
	expected.insert(x1, pt_x1);
	expected.insert(x2, pt_x2);
	expected.insert(l1, Point2(1.0, 6.0));
	CHECK(assert_equal(expected, *actual, 1e-5));
}

/* ********************************************************************* */
namespace sqp_test1 {

	// binary constraint between landmarks
	/** g(x) = x-y = 0 */
	Vector g(const Config2D& config, const list<Symbol>& keys) {
		Point2 pt1, pt2;
		pt1 = config[simulated2D::PointKey(keys.front().index())];
		pt2 = config[simulated2D::PointKey(keys.back().index())];
		return Vector_(2, pt1.x() - pt2.x(), pt1.y() - pt2.y());
	}

	/** jacobian at l1 */
	Matrix G1(const Config2D& config, const list<Symbol>& keys) {
		return eye(2);
	}

	/** jacobian at l2 */
	Matrix G2(const Config2D& config, const list<Symbol>& keys) {
		return -1 * eye(2);
	}

} // \namespace sqp_test1

//namespace sqp_test2 {
//
//	// Unary Constraint on x1
//	/** g(x) = x -[1;1] = 0 */
//	Vector g(const Config2D& config, const list<Symbol>& keys) {
//		return config[keys.front()] - Vector_(2, 1.0, 1.0);
//	}
//
//	/** jacobian at x1 */
//	Matrix G(const Config2D& config, const list<Symbol>& keys) {
//		return eye(2);
//	}
//
//} // \namespace sqp_test2


typedef NonlinearConstraint2<
	Config2D, simulated2D::PointKey, Point2, simulated2D::PointKey, Point2> NLC2;

/* *********************************************************************
 *  Version that actually uses nonlinear equality constraints
 *  to to perform optimization.  Same as above, but no
 *  equality constraint on x2, and two landmarks that
 *  should be the same. Note that this is a linear system,
 *  so it will converge in one step.
 */
TEST (SQP, two_pose ) {
	bool verbose = false;

	// create the graph
	shared_ptr<NLGraph> graph(new NLGraph);

	// add the constraints on the ends
	// position (1, 1) constraint for x1
	// position (5, 6) constraint for x2
	simulated2D::PoseKey x1(1), x2(2);
	simulated2D::PointKey l1(1), l2(2);
	Point2 pt_x1(1.0, 1.0),
		   pt_x2(5.0, 6.0);
	shared_ptr<NLE> ef1(new NLE(x1, pt_x1));
	graph->push_back(ef1);

	// measurement from x1 to l1
	Point2 z1(0.0, 5.0);
	sharedGaussian sigma(noiseModel::Isotropic::Sigma(2, 0.1));
	shared f1(new simulated2D::Measurement(z1, sigma, x1,l1));
	graph->push_back(f1);

	// measurement from x2 to l2
	Point2 z2(-4.0, 0.0);
	shared f2(new simulated2D::Measurement(z2, sigma, x2,l2));
	graph->push_back(f2);

	// equality constraint between l1 and l2
	list<Symbol> keys2; keys2 += "l1", "l2";
	boost::shared_ptr<NLC2 > c2(new NLC2(
					boost::bind(sqp_test1::g, _1, keys2),
					l1, boost::bind(sqp_test1::G1, _1, keys2),
					l2, boost::bind(sqp_test1::G2, _1, keys2),
					2, "L1"));
	graph->push_back(c2);

	// create an initial estimate
	shared_ptr<Config2D> initialEstimate(new Config2D);
	initialEstimate->insert(x1, pt_x1);
	initialEstimate->insert(x2, Point2());
	initialEstimate->insert(l1, Point2(1.0, 6.0)); // ground truth
	initialEstimate->insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame

	// create an initial estimate for the lagrange multiplier
	shared_ptr<VectorConfig> initLagrange(new VectorConfig);
	initLagrange->insert(LamKey(1), Vector_(2, 1.0, 1.0)); // connect the landmarks

	// create state config variables and initialize them
	Config2D state(*initialEstimate);
	VectorConfig state_lambda(*initLagrange);

	// optimization loop
	int maxIt = 1;
	for (int i = 0; i<maxIt; ++i) {

 		// linearize the graph
		GaussianFactorGraph fg;
		typedef FactorGraph<NonlinearFactor<Config2D> >::const_iterator const_iterator;
		// iterate over all factors
		for (const_iterator factor = graph->begin(); factor < graph->end(); factor++) {
			const shared_ptr<NLC2> constraint = boost::shared_dynamic_cast<NLC2>(*factor);
			if (constraint == NULL) {
				// if a regular factor, linearize using the default linearization
				GaussianFactor::shared_ptr f = (*factor)->linearize(state);
				fg.push_back(f);
			} else {
				// if a constraint, linearize using the constraint method (2 configs)
				GaussianFactor::shared_ptr f, c;
				boost::tie(f,c) = constraint->linearize(state, state_lambda);
				fg.push_back(f);
				fg.push_back(c);
			}
		}

		if (verbose) fg.print("Linearized graph");

		// create an ordering
		Ordering ordering;
		ordering += "x1", "x2", "l1", "l2", "L1";

		// optimize linear graph to get full delta config
		VectorConfig delta = fg.optimize(ordering);
		if (verbose) delta.print("Delta Config");

		// update both state variables
		state = expmap(state, delta);
		if (verbose) state.print("newState");
		state_lambda = expmap(state_lambda, delta);
		if (verbose) state_lambda.print("newStateLam");
	}

	// verify
	Config2D expected;
	expected.insert(x1, pt_x1);
	expected.insert(l1, Point2(1.0, 6.0));
	expected.insert(l2, Point2(1.0, 6.0));
	expected.insert(x2, Point2(5.0, 6.0));
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

using namespace gtsam::visualSLAM;
using namespace boost;

// typedefs for visual SLAM example
typedef visualSLAM::Config VConfig;
typedef visualSLAM::Graph VGraph;
typedef boost::shared_ptr<ProjectionFactor> shared_vf;
typedef NonlinearOptimizer<VGraph,VConfig> VOptimizer;
typedef SQPOptimizer<VGraph, VConfig> VSOptimizer;
typedef NonlinearConstraint2<
	VConfig, visualSLAM::PointKey, Pose3, visualSLAM::PointKey, Pose3> VNLC2;


/**
 * Ground truth for a visual SLAM example with stereo vision
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
	boost::shared_ptr<VConfig> truthConfig(new Config);
	truthConfig->insert(1, camera1.pose());
	truthConfig->insert(2, camera2.pose());
	truthConfig->insert(1, landmark);

	// create graph
	shared_ptr<VGraph> graph(new VGraph());

	// create equality constraints for poses
	graph->push_back(shared_ptr<PoseConstraint>(new PoseConstraint(1, camera1.pose())));
	graph->push_back(shared_ptr<PoseConstraint>(new PoseConstraint(2, camera2.pose())));

	// create VSLAM factors
	Point2 z1 = camera1.project(landmark);
	if (verbose) z1.print("z1");
	shared_vf vf1(new ProjectionFactor(z1, 1.0, 1, 1, shK));
	graph->push_back(vf1);
	Point2 z2 = camera2.project(landmark);
	if (verbose) z2.print("z2");
	shared_vf vf2(new ProjectionFactor(z2, 1.0, 2, 1, shK));
	graph->push_back(vf2);

	if (verbose) graph->print("Graph after construction");

	// create ordering
	shared_ptr<Ordering> ord(new Ordering());
	*ord += "x1", "x2", "l1";

	// create optimizer
	VOptimizer::shared_solver solver(new VOptimizer::solver(ord));
	VOptimizer optimizer(graph, truthConfig, solver);

	// optimize
	VOptimizer afterOneIteration = optimizer.iterate();

	// verify
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

	// check if correct
	if (verbose) afterOneIteration.config()->print("After iteration");
	CHECK(assert_equal(*truthConfig,*(afterOneIteration.config())));
}


/* *********************************************************************
 * Ground truth for a visual SLAM example with stereo vision
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
	boost::shared_ptr<Config> truthConfig(new Config);
	truthConfig->insert(1, camera1.pose());
	truthConfig->insert(2, camera2.pose());
	truthConfig->insert(1, landmark);

	// create config
	boost::shared_ptr<Config> noisyConfig(new Config);
	noisyConfig->insert(1, camera1.pose());
	noisyConfig->insert(2, camera2.pose());
	noisyConfig->insert(1, landmarkNoisy);

	// create graph
	shared_ptr<Graph> graph(new Graph());

	// create equality constraints for poses
	graph->push_back(shared_ptr<PoseConstraint>(new PoseConstraint(1, camera1.pose())));
	graph->push_back(shared_ptr<PoseConstraint>(new PoseConstraint(2, camera2.pose())));

	// create VSLAM factors
	Point2 z1 = camera1.project(landmark);
	if (verbose) z1.print("z1");
	shared_vf vf1(new ProjectionFactor(z1, 1.0, 1, 1, shK));
	graph->push_back(vf1);
	Point2 z2 = camera2.project(landmark);
	if (verbose) z2.print("z2");
	shared_vf vf2(new ProjectionFactor(z2, 1.0, 2, 1, shK));
	graph->push_back(vf2);

	if (verbose)  {
		graph->print("Graph after construction");
		noisyConfig->print("Initial config");
	}

	// create ordering
	shared_ptr<Ordering> ord(new Ordering());
	*ord += "x1", "x2", "l1";

	// create optimizer
	VOptimizer::shared_solver solver(new VOptimizer::solver(ord));
	VOptimizer optimizer0(graph, noisyConfig, solver);

	if (verbose)
		cout << "Initial Error: " << optimizer0.error() << endl;

	// use Levenberg-Marquardt optimization
	double relThresh = 1e-5, absThresh = 1e-5;
	VOptimizer optimizer(optimizer0.levenbergMarquardt(relThresh, absThresh, VOptimizer::SILENT));

	// verify
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

	// check if correct
	if (verbose) {
		optimizer.config()->print("After iteration");
		cout << "Final error: " << optimizer.error() << endl;
	}
	CHECK(assert_equal(*truthConfig,*(optimizer.config())));
}

/* ********************************************************************* */
namespace sqp_stereo {

	// binary constraint between landmarks
	/** g(x) = x-y = 0 */
	Vector g(const Config& config, const list<Symbol>& keys) {
		return config[PointKey(keys.front().index())].vector()
				- config[PointKey(keys.back().index())].vector();
	}

	/** jacobian at l1 */
	Matrix G1(const Config& config, const list<Symbol>& keys) {
		return eye(3);
	}

	/** jacobian at l2 */
	Matrix G2(const Config& config, const list<Symbol>& keys) {
		return -1.0 * eye(3);
	}

} // \namespace sqp_stereo

/* ********************************************************************* */
VGraph stereoExampleGraph() {
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
	VGraph graph;

	// create equality constraints for poses
	graph.push_back(shared_ptr<PoseConstraint>(new PoseConstraint(1, camera1.pose())));
	graph.push_back(shared_ptr<PoseConstraint>(new PoseConstraint(2, camera2.pose())));

	// create  factors
	Point2 z1 = camera1.project(landmark1);
	shared_vf vf1(new ProjectionFactor(z1, 1.0, 1, 1, shK));
	graph.push_back(vf1);
	Point2 z2 = camera2.project(landmark2);
	shared_vf vf2(new ProjectionFactor(z2, 1.0, 2, 2, shK));
	graph.push_back(vf2);

	// create the binary equality constraint between the landmarks
	// NOTE: this is really just a linear constraint that is exactly the same
	// as the previous examples
	list<Symbol> keys; keys += "l1", "l2";
	visualSLAM::PointKey l1(1), l2(2);
	shared_ptr<VNLC2> c2(
			new VNLC2(boost::bind(sqp_stereo::g, _1, keys),
					 l1, boost::bind(sqp_stereo::G1, _1, keys),
					 l2, boost::bind(sqp_stereo::G2, _1, keys),
					 3, "L12"));
	graph.push_back(c2);

	return graph;
}

/* ********************************************************************* */
boost::shared_ptr<VConfig> stereoExampleTruthConfig() {
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
	boost::shared_ptr<Config> truthConfig(new Config);
	truthConfig->insert(1, camera1.pose());
	truthConfig->insert(2, camera2.pose());
	truthConfig->insert(1, landmark1);
	truthConfig->insert(2, landmark2); // create two landmarks in same place

	return truthConfig;
}

/* *********************************************************************
 * SQP version of the above stereo example,
 * with the initial case as the ground truth
 */
TEST (SQP, stereo_sqp ) {
	bool verbose = false;

	// get a graph
	VGraph graph = stereoExampleGraph();
	if (verbose) graph.print("Graph after construction");

	// get the truth config
	boost::shared_ptr<VConfig> truthConfig = stereoExampleTruthConfig();

	// create ordering
	Ordering ord;
	ord += "x1", "x2", "l1", "l2";

	// create optimizer
	VSOptimizer optimizer(graph, ord, truthConfig);

	// optimize
	VSOptimizer afterOneIteration = optimizer.iterate();

	// check if correct
	CHECK(assert_equal(*truthConfig,*(afterOneIteration.config())));
}

/* *********************************************************************
 * SQP version of the above stereo example,
 * with noise in the initial estimate
 */
TEST (SQP, stereo_sqp_noisy ) {
	bool verbose = false;

	// get a graph
	Graph graph = stereoExampleGraph();

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
	boost::shared_ptr<Config> initConfig(new Config);
	initConfig->insert(1, pose1);
	initConfig->insert(2, pose2);
	initConfig->insert(1, landmark1);
	initConfig->insert(2, landmark2); // create two landmarks in same place

	// create ordering
	Ordering ord;
	ord += "x1", "x2", "l1", "l2";

	// create optimizer
	VSOptimizer optimizer(graph, ord, initConfig);

	// optimize
	double start_error = optimizer.error();
	int maxIt = 2;
	for (int i=0; i<maxIt; ++i) {
		if (verbose) cout << "\n ************************** \n"
						  << " Iteration: " << i << endl;
		//if (verbose) optimizer.graph()->print();
		if (verbose) optimizer.config()->print();
		if (verbose)
			optimizer = optimizer.iterate(VSOptimizer::FULL);
		else
			optimizer = optimizer.iterate(VSOptimizer::SILENT);
	}

	if (verbose) cout << "Initial Error: " << start_error << "\n"
					  << "Final Error:   " << optimizer.error() << endl;

	// get the truth config
	boost::shared_ptr<Config> truthConfig = stereoExampleTruthConfig();

	if (verbose) {
		initConfig->print("Initial Config");
		truthConfig->print("Truth Config");
		optimizer.config()->print("After optimization");
	}

	// check if correct
	CHECK(assert_equal(*truthConfig,*(optimizer.config())));
}

/* *********************************************************************
 * SQP version of the above stereo example,
 * with noise in the initial estimate and manually specified
 * lagrange multipliers
 */
TEST (SQP, stereo_sqp_noisy_manualLagrange ) {
	bool verbose = false;

	// get a graph
	Graph graph = stereoExampleGraph();

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
	boost::shared_ptr<Config> initConfig(new Config);
	initConfig->insert(1, pose1);
	initConfig->insert(2, pose2);
	initConfig->insert(1, landmark1);
	initConfig->insert(2, landmark2); // create two landmarks in same place

	// create ordering with lagrange multiplier included
	Ordering ord;
	ord += "x1", "x2", "l1", "l2", "L12";

	// create lagrange multipliers
	VSOptimizer::shared_vconfig initLagrangeConfig(new VectorConfig);
	initLagrangeConfig->insert("L12", Vector_(3, 0.0, 0.0, 0.0));

	// create optimizer
	VSOptimizer optimizer(graph, ord, initConfig, initLagrangeConfig);

	// optimize
	double start_error = optimizer.error();
	int maxIt = 5;
	for (int i=0; i<maxIt; ++i) {
		if (verbose) {
			cout << "\n ************************** \n"
				 << " Iteration: " << i << endl;
			optimizer.config()->print("Config Before Iteration");
			optimizer.configLagrange()->print("Lagrange Before Iteration");
			optimizer = optimizer.iterate(VSOptimizer::FULL);
		}
		else
			optimizer = optimizer.iterate(VSOptimizer::SILENT);
	}

	if (verbose) cout << "Initial Error: " << start_error << "\n"
					  << "Final Error:   " << optimizer.error() << endl;

	// get the truth config
	boost::shared_ptr<Config> truthConfig = stereoExampleTruthConfig();

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
