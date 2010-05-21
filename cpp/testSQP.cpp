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

#define GTSAM_MAGIC_KEY

#include <Point2.h>
#include <Pose3.h>
#include <GaussianFactorGraph.h>
#include <NonlinearOptimizer.h>
#include <simulated2D.h>
#include <Ordering.h>
#include <visualSLAM.h>

// templated implementations
#include <NonlinearFactorGraph-inl.h>
#include <NonlinearConstraint-inl.h>
#include <NonlinearOptimizer-inl.h>
#include <TupleConfig-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost;
using namespace boost::assign;

// Models to use
SharedDiagonal probModel1 = sharedSigma(1,1.0);
SharedDiagonal probModel2 = sharedSigma(2,1.0);
SharedDiagonal constraintModel1 = noiseModel::Constrained::All(1);

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

/* ********************************************************************* */

// Basic configs
typedef LieConfig<LagrangeKey, Vector> LagrangeConfig;

// full components
typedef TupleConfig3<LieConfig<simulated2D::PoseKey, Point2>,
					 LieConfig<simulated2D::PointKey, Point2>,
					 LieConfig<LagrangeKey, Vector> > Config2D;
//typedef TupleConfig<LagrangeConfig, TupleConfigEnd<simulated2D::Config> > Config2D;
typedef NonlinearFactorGraph<Config2D> Graph2D;
typedef NonlinearEquality<Config2D, simulated2D::PoseKey, Point2> NLE;
typedef boost::shared_ptr<simulated2D::GenericMeasurement<Config2D> > shared;
typedef NonlinearOptimizer<Graph2D, Config2D> Optimizer;

/*
 * Determining a ground truth linear system
 * with two poses seeing one landmark, with each pose
 * constrained to a particular value
 */
TEST (SQP, two_pose_truth ) {
	bool verbose = false;

	// create a graph
	shared_ptr<Graph2D> graph(new Graph2D);

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
	SharedGaussian sigma(noiseModel::Isotropic::Sigma(2, 0.1));
	shared f1(new simulated2D::GenericMeasurement<Config2D>(z1, sigma, x1,l1));
	graph->push_back(f1);

	// measurement from x2 to l1
	Point2 z2(-4.0, 0.0);
	shared f2(new simulated2D::GenericMeasurement<Config2D>(z2, sigma, x2,l1));
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
	Vector g(const Config2D& config, const list<simulated2D::PointKey>& keys) {
		Point2 pt1, pt2;
		pt1 = config[simulated2D::PointKey(keys.front().index())];
		pt2 = config[simulated2D::PointKey(keys.back().index())];
		return Vector_(2, pt1.x() - pt2.x(), pt1.y() - pt2.y());
	}

	/** jacobian at l1 */
	Matrix G1(const Config2D& config, const list<simulated2D::PointKey>& keys) {
		return eye(2);
	}

	/** jacobian at l2 */
	Matrix G2(const Config2D& config, const list<simulated2D::PointKey>& keys) {
		return -1 * eye(2);
	}

} // \namespace sqp_test1

namespace sqp_test2 {

	// Unary Constraint on x1
	/** g(x) = x -[1;1] = 0 */
	Vector g(const Config2D& config, const list<simulated2D::PoseKey>& keys) {
		Point2 x = config[keys.front()];
		return Vector_(2, x.x() - 1.0, x.y() - 1.0);
	}

	/** jacobian at x1 */
	Matrix G(const Config2D& config, const list<simulated2D::PoseKey>& keys) {
		return eye(2);
	}

} // \namespace sqp_test2


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
	shared_ptr<Graph2D> graph(new Graph2D);

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
	SharedGaussian sigma(noiseModel::Isotropic::Sigma(2, 0.1));
	shared f1(new simulated2D::GenericMeasurement<Config2D>(z1, sigma, x1,l1));
	graph->push_back(f1);

	// measurement from x2 to l2
	Point2 z2(-4.0, 0.0);
	shared f2(new simulated2D::GenericMeasurement<Config2D>(z2, sigma, x2,l2));
	graph->push_back(f2);

	// equality constraint between l1 and l2
	LagrangeKey L1(1);
	list<simulated2D::PointKey> keys2; keys2 += l1, l2;
	boost::shared_ptr<NLC2 > c2(new NLC2(
					boost::bind(sqp_test1::g, _1, keys2),
					l1, boost::bind(sqp_test1::G1, _1, keys2),
					l2, boost::bind(sqp_test1::G2, _1, keys2),
					2, L1));
	graph->push_back(c2);

	if (verbose) graph->print("Initial nonlinear graph with constraints");

	// create an initial estimate
	shared_ptr<Config2D> initialEstimate(new Config2D);
	initialEstimate->insert(x1, pt_x1);
	initialEstimate->insert(x2, Point2());
	initialEstimate->insert(l1, Point2(1.0, 6.0)); // ground truth
	initialEstimate->insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert(L1, Vector_(2, 1.0, 1.0)); // connect the landmarks

	// create state config variables and initialize them
	Config2D state(*initialEstimate);

	// linearize the graph
	boost::shared_ptr<GaussianFactorGraph> fg = graph->linearize(state);

	if (verbose) fg->print("Linearized graph");

	// create an ordering
	Ordering ordering;
	ordering += "x1", "x2", "l1", "l2", "L1";

	// optimize linear graph to get full delta config
	GaussianBayesNet cbn = fg->eliminate(ordering);
	if (verbose) cbn.print("ChordalBayesNet");

	VectorConfig delta = optimize(cbn); //fg.optimize(ordering);
	if (verbose) delta.print("Delta Config");

	// update both state variables
	state = expmap(state, delta);
	if (verbose) state.print("newState");

	// verify
	Config2D expected;
	expected.insert(x1, pt_x1);
	expected.insert(l1, Point2(1.0, 6.0));
	expected.insert(l2, Point2(1.0, 6.0));
	expected.insert(x2, Point2(5.0, 6.0));
	expected.insert(L1, Vector_(2, 6.0, 7.0));
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
typedef TypedSymbol<Pose3, 'x'> Pose3Key;
typedef TypedSymbol<Point3, 'l'> Point3Key;
typedef TupleConfig3<LieConfig<LagrangeKey, Vector>,
					 LieConfig<Pose3Key, Pose3>,
					 LieConfig<Point3Key, Point3> > VConfig;
typedef NonlinearFactorGraph<VConfig> VGraph;
typedef boost::shared_ptr<GenericProjectionFactor<VConfig> > shared_vf;
typedef NonlinearOptimizer<VGraph,VConfig> VOptimizer;
typedef NonlinearConstraint2<
	VConfig, visualSLAM::PointKey, Pose3, visualSLAM::PointKey, Pose3> VNLC2;
typedef NonlinearEquality<VConfig, Pose3Key, Pose3> Pose3Constraint;

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
	boost::shared_ptr<VConfig> truthConfig(new VConfig);
	truthConfig->insert(Pose3Key(1), camera1.pose());
	truthConfig->insert(Pose3Key(2), camera2.pose());
	truthConfig->insert(Point3Key(1), landmark);

	// create graph
	shared_ptr<VGraph> graph(new VGraph());

	// create equality constraints for poses
	graph->push_back(shared_ptr<Pose3Constraint>(new Pose3Constraint(Pose3Key(1), camera1.pose())));
	graph->push_back(shared_ptr<Pose3Constraint>(new Pose3Constraint(Pose3Key(2), camera2.pose())));

	// create VSLAM factors
	Point2 z1 = camera1.project(landmark);
	if (verbose) z1.print("z1");
	SharedDiagonal vmodel = noiseModel::Unit::Create(3);
	//ProjectionFactor test_vf(z1, vmodel, Pose3Key(1), Point3Key(1), shK);
	shared_vf vf1(new GenericProjectionFactor<VConfig>(
			z1, vmodel, Pose3Key(1), Point3Key(1), shK));
	graph->push_back(vf1);
	Point2 z2 = camera2.project(landmark);
	if (verbose) z2.print("z2");
	shared_vf vf2(new GenericProjectionFactor<VConfig>(
			z2, vmodel, Pose3Key(2), Point3Key(1), shK));
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
	boost::shared_ptr<VConfig> truthConfig(new VConfig);
	truthConfig->insert(Pose3Key(1), camera1.pose());
	truthConfig->insert(Pose3Key(2), camera2.pose());
	truthConfig->insert(Point3Key(1), landmark);

	// create config
	boost::shared_ptr<VConfig> noisyConfig(new VConfig);
	noisyConfig->insert(Pose3Key(1), camera1.pose());
	noisyConfig->insert(Pose3Key(2), camera2.pose());
	noisyConfig->insert(Point3Key(1), landmarkNoisy);

	// create graph
	shared_ptr<VGraph> graph(new VGraph());

	// create equality constraints for poses
	graph->push_back(shared_ptr<Pose3Constraint>(new Pose3Constraint(Pose3Key(1), camera1.pose())));
	graph->push_back(shared_ptr<Pose3Constraint>(new Pose3Constraint(Pose3Key(2), camera2.pose())));

	// create VSLAM factors
	Point2 z1 = camera1.project(landmark);
	if (verbose) z1.print("z1");
	SharedDiagonal vmodel = noiseModel::Unit::Create(3);
	shared_vf vf1(new GenericProjectionFactor<VConfig>(
				z1, vmodel, Pose3Key(1), Point3Key(1), shK));
	graph->push_back(vf1);
	Point2 z2 = camera2.project(landmark);
	if (verbose) z2.print("z2");
	shared_vf vf2(new GenericProjectionFactor<VConfig>(
				z2, vmodel, Pose3Key(2), Point3Key(1), shK));
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
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-5);

	// check if correct
	if (verbose) {
		optimizer.config()->print("After iteration");
		cout << "Final error: " << optimizer.error() << endl;
	}
	CHECK(assert_equal(*truthConfig,*(optimizer.config()), 1e-5));
}

/* ********************************************************************* */
namespace sqp_stereo {

	// binary constraint between landmarks
	/** g(x) = x-y = 0 */
	Vector g(const VConfig& config, const list<Point3Key>& keys) {
		return config[keys.front()].vector()
				- config[keys.back()].vector();
	}

	/** jacobian at l1 */
	Matrix G1(const VConfig& config, const list<Point3Key>& keys) {
		return eye(3);
	}

	/** jacobian at l2 */
	Matrix G2(const VConfig& config, const list<Point3Key>& keys) {
		return -1.0 * eye(3);
	}

} // \namespace sqp_stereo

/* ********************************************************************* */
boost::shared_ptr<VGraph> stereoExampleGraph() {
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
	shared_ptr<VGraph> graph(new VGraph);

	// create equality constraints for poses
	graph->push_back(shared_ptr<Pose3Constraint>(new Pose3Constraint(Pose3Key(1), camera1.pose())));
	graph->push_back(shared_ptr<Pose3Constraint>(new Pose3Constraint(Pose3Key(2), camera2.pose())));

	// create  factors
	Point2 z1 = camera1.project(landmark1);
	SharedDiagonal vmodel = noiseModel::Unit::Create(3);
	shared_vf vf1(new GenericProjectionFactor<VConfig>(
				z1, vmodel, Pose3Key(1), Point3Key(1), shK));
	graph->push_back(vf1);
	Point2 z2 = camera2.project(landmark2);
	shared_vf vf2(new GenericProjectionFactor<VConfig>(
				z2, vmodel, Pose3Key(2), Point3Key(2), shK));
	graph->push_back(vf2);

	// create the binary equality constraint between the landmarks
	// NOTE: this is really just a linear constraint that is exactly the same
	// as the previous examples
	visualSLAM::PointKey l1(1), l2(2);
	list<Point3Key> keys; keys += l1, l2;
	LagrangeKey L12(12);
	shared_ptr<VNLC2> c2(
			new VNLC2(boost::bind(sqp_stereo::g, _1, keys),
					 l1, boost::bind(sqp_stereo::G1, _1, keys),
					 l2, boost::bind(sqp_stereo::G2, _1, keys),
					 3, L12));
	graph->push_back(c2);

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
	boost::shared_ptr<VConfig> truthConfig(new VConfig);
	truthConfig->insert(Pose3Key(1), camera1.pose());
	truthConfig->insert(Pose3Key(2), camera2.pose());
	truthConfig->insert(Point3Key(1), landmark1);
	truthConfig->insert(Point3Key(2), landmark2); // create two landmarks in same place
	//truthConfig->insert(LagrangeKey(12), Vector_(3, 1.0, 1.0, 1.0));

	return truthConfig;
}

/* *********************************************************************
 * SQP version of the above stereo example,
 * with the initial case as the ground truth
 */
TEST (SQP, stereo_sqp ) {
	bool verbose = false;

	// get a graph
	boost::shared_ptr<VGraph> graph = stereoExampleGraph();
	if (verbose) graph->print("Graph after construction");

	// get the truth config
	boost::shared_ptr<VConfig> truthConfig = stereoExampleTruthConfig();
	truthConfig->insert(LagrangeKey(12), Vector_(3, 1.0, 1.0, 1.0));

	// create ordering
	shared_ptr<Ordering> ord(new Ordering());
	*ord += "x1", "x2", "l1", "l2", "L12";
	VOptimizer::shared_solver solver(new VOptimizer::solver(ord));

	// create optimizer
	VOptimizer optimizer(graph, truthConfig, solver);

	// optimize
	VOptimizer afterOneIteration = optimizer.iterate();

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
	boost::shared_ptr<VGraph> graph = stereoExampleGraph();

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
	boost::shared_ptr<VConfig> initConfig(new VConfig);
	initConfig->insert(Pose3Key(1), pose1);
	initConfig->insert(Pose3Key(2), pose2);
	initConfig->insert(Point3Key(1), landmark1);
	initConfig->insert(Point3Key(2), landmark2); // create two landmarks in same place
	initConfig->insert(LagrangeKey(12), Vector_(3, 1.0, 1.0, 1.0));

	// create ordering
	shared_ptr<Ordering> ord(new Ordering());
	*ord += "x1", "x2", "l1", "l2", "L12";
	VOptimizer::shared_solver solver(new VOptimizer::solver(ord));

	// create optimizer
	VOptimizer optimizer(graph, initConfig, solver);

	// optimize
	VOptimizer *pointer = new VOptimizer(optimizer);
	for (int i=0;i<1;i++) {
		VOptimizer* newOptimizer = new VOptimizer(pointer->iterateLM());
		delete pointer;
		pointer = newOptimizer;
	}
	VOptimizer::shared_config actual = pointer->config();
	delete(pointer);

	// get the truth config
	boost::shared_ptr<VConfig> truthConfig = stereoExampleTruthConfig();
	truthConfig->insert(LagrangeKey(12), Vector_(3, 0.0, 1.0, 1.0));

	// check if correct
	CHECK(assert_equal(*truthConfig,*actual, 1e-5));
}

static SharedGaussian sigma(noiseModel::Isotropic::Sigma(1,0.1));

// typedefs
//typedef simulated2D::Config Config2D;
//typedef boost::shared_ptr<Config2D> shared_config;
//typedef NonlinearFactorGraph<Config2D> NLGraph;
//typedef boost::shared_ptr<NonlinearFactor<Config2D> > shared;

namespace map_warp_example {
typedef NonlinearConstraint1<
	Config2D, simulated2D::PoseKey, Point2> NLC1;
//typedef NonlinearConstraint2<
//	Config2D, simulated2D::PointKey, Point2, simulated2D::PointKey, Point2> NLC2;
} // \namespace map_warp_example

/* ********************************************************************* */
// Example that moves two separate maps into the same frame of reference
// Note that this is a linear example, so it should converge in one step
/* ********************************************************************* */

namespace sqp_LinearMapWarp2 {
// binary constraint between landmarks
/** g(x) = x-y = 0 */
Vector g_func(const Config2D& config, const simulated2D::PointKey& key1, const simulated2D::PointKey& key2) {
	Point2 p = config[key1]-config[key2];
	return Vector_(2, p.x(), p.y());
}

/** jacobian at l1 */
Matrix jac_g1(const Config2D& config) {
	return eye(2);
}

/** jacobian at l2 */
Matrix jac_g2(const Config2D& config) {
	return -1*eye(2);
}
} // \namespace sqp_LinearMapWarp2

namespace sqp_LinearMapWarp1 {
// Unary Constraint on x1
/** g(x) = x -[1;1] = 0 */
Vector g_func(const Config2D& config, const simulated2D::PoseKey& key) {
	Point2 p = config[key]-Point2(1.0, 1.0);
	return Vector_(2, p.x(), p.y());
}

/** jacobian at x1 */
Matrix jac_g(const Config2D& config) {
	return eye(2);
}
} // \namespace sqp_LinearMapWarp12

//typedef NonlinearOptimizer<NLGraph, Config2D> Optimizer;

/**
 * Creates the graph with each robot seeing the landmark, and it is
 * known that it is the same landmark
 */
boost::shared_ptr<Graph2D> linearMapWarpGraph() {
	using namespace map_warp_example;
	// keys
	simulated2D::PoseKey x1(1), x2(2);
	simulated2D::PointKey l1(1), l2(2);

	// constant constraint on x1
	LagrangeKey L1(1);
	shared_ptr<NLC1> c1(new NLC1(boost::bind(sqp_LinearMapWarp1::g_func, _1, x1),
							x1, boost::bind(sqp_LinearMapWarp1::jac_g, _1),
							2, L1));

	// measurement from x1 to l1
	Point2 z1(0.0, 5.0);
	shared f1(new simulated2D::GenericMeasurement<Config2D>(z1, sigma, x1,l1));

	// measurement from x2 to l2
	Point2 z2(-4.0, 0.0);
	shared f2(new simulated2D::GenericMeasurement<Config2D>(z2, sigma, x2,l2));

	// equality constraint between l1 and l2
	LagrangeKey L12(12);
	shared_ptr<NLC2> c2 (new NLC2(
			boost::bind(sqp_LinearMapWarp2::g_func, _1, l1, l2),
			l1, boost::bind(sqp_LinearMapWarp2::jac_g1, _1),
			l2, boost::bind(sqp_LinearMapWarp2::jac_g2, _1),
			2, L12));

	// construct the graph
	boost::shared_ptr<Graph2D> graph(new Graph2D());
	graph->push_back(c1);
	graph->push_back(c2);
	graph->push_back(f1);
	graph->push_back(f2);

	return graph;
}

/* ********************************************************************* */
TEST ( SQPOptimizer, map_warp_initLam ) {
	// get a graph
	boost::shared_ptr<Graph2D> graph = linearMapWarpGraph();

	// keys
	simulated2D::PoseKey x1(1), x2(2);
	simulated2D::PointKey l1(1), l2(2);
	LagrangeKey L1(1), L12(12);

	// create an initial estimate
	shared_ptr<Config2D> initialEstimate(new Config2D);
	initialEstimate->insert(x1, Point2(1.0, 1.0));
	initialEstimate->insert(l1, Point2(1.0, 6.0));
	initialEstimate->insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert(x2, Point2(0.0, 0.0)); // other pose starts at origin
	initialEstimate->insert(L12, Vector_(2, 1.0, 1.0));
	initialEstimate->insert(L1, Vector_(2, 1.0, 1.0));

	// create an ordering
	shared_ptr<Ordering> ordering(new Ordering());
	*ordering += "x1", "x2", "l1", "l2", "L12", "L1";

	// create an optimizer
	Optimizer::shared_solver solver(new Optimizer::solver(ordering));
	Optimizer optimizer(graph, initialEstimate, solver);

	// perform an iteration of optimization
	Optimizer oneIteration = optimizer.iterate(Optimizer::SILENT);

	// get the config back out and verify
	Config2D actual = *(oneIteration.config());
	Config2D expected;
	expected.insert(x1, Point2(1.0, 1.0));
	expected.insert(l1, Point2(1.0, 6.0));
	expected.insert(l2, Point2(1.0, 6.0));
	expected.insert(x2, Point2(5.0, 6.0));
	expected.insert(L1, Vector_(2, 1.0, 1.0));
	expected.insert(L12, Vector_(2, 6.0, 7.0));
	CHECK(assert_equal(expected, actual));
}

/* ********************************************************************* */
// This is an obstacle avoidance demo, where there is a trajectory of
// three points, where there is a circular obstacle in the middle.  There
// is a binary inequality constraint connecting the obstacle to the
// states, which enforces a minimum distance.
/* ********************************************************************* */

//typedef NonlinearConstraint2<Config2D, PoseKey, Point2, PointKey, Point2> AvoidConstraint;
//typedef shared_ptr<AvoidConstraint> shared_a;
//typedef NonlinearEquality<Config2D, simulated2D::PoseKey, Point2> PoseNLConstraint;
//typedef shared_ptr<PoseNLConstraint> shared_pc;
//typedef NonlinearEquality<Config2D, simulated2D::PointKey, Point2> ObstacleConstraint;
//typedef shared_ptr<ObstacleConstraint> shared_oc;
//
//
//namespace sqp_avoid1 {
//// avoidance radius
//double radius = 1.0;
//
//// binary avoidance constraint
///** g(x) = ||x2-obs||^2 - radius^2 > 0 */
//Vector g_func(const Config2D& config, const PoseKey& x, const PointKey& obs) {
//	double dist2 = config[x].dist(config[obs]);
//	double thresh = radius*radius;
//	return Vector_(1, dist2-thresh);
//}
//
///** jacobian at pose */
//Matrix jac_g1(const Config2D& config, const PoseKey& x, const PointKey& obs) {
//	Point2 p = config[x]-config[obs];
//	return Matrix_(1,2, 2.0*p.x(), 2.0*p.y());
//}
//
///** jacobian at obstacle */
//Matrix jac_g2(const Config2D& config, const PoseKey& x, const PointKey& obs) {
//	Point2 p = config[x]-config[obs];
//	return Matrix_(1,2, -2.0*p.x(), -2.0*p.y());
//}
//}
//
//pair<NLGraph, Config2D> obstacleAvoidGraph() {
//	// Keys
//	PoseKey x1(1), x2(2), x3(3);
//	PointKey l1(1);
//	LagrangeKey L20(20);
//
//	// Constrained Points
//	Point2 pt_x1,
//		   pt_x3(10.0, 0.0),
//		   pt_l1(5.0, -0.5);
//
//	shared_pc e1(new PoseNLConstraint(x1, pt_x1));
//	shared_pc e2(new PoseNLConstraint(x3, pt_x3));
//	shared_oc e3(new ObstacleConstraint(l1, pt_l1));
//
//	// measurement from x1 to x2
//	Point2 x1x2(5.0, 0.0);
//	shared f1(new simulated2D::Odometry(x1x2, sigma, 1,2));
//
//	// measurement from x2 to x3
//	Point2 x2x3(5.0, 0.0);
//	shared f2(new simulated2D::Odometry(x2x3, sigma, 2,3));
//
//	// create a binary inequality constraint that forces the middle point away from
//	//  the obstacle
//	shared_a c1(new AvoidConstraint(boost::bind(sqp_avoid1::g_func, _1, x2, l1),
//							x2, boost::bind(sqp_avoid1::jac_g1, _1, x2, l1),
//						    l1,boost::bind(sqp_avoid1::jac_g2, _1, x2, l1),
//						    1, L20, false));
//
//	// construct the graph
//	NLGraph graph;
//	graph.push_back(e1);
//	graph.push_back(e2);
//	graph.push_back(e3);
//	graph.push_back(c1);
//	graph.push_back(f1);
//	graph.push_back(f2);
//
//	// make a config of the fixed values, for convenience
//	Config2D config;
//	config.insert(x1, pt_x1);
//	config.insert(x3, pt_x3);
//	config.insert(l1, pt_l1);
//
//	return make_pair(graph, config);
//}
//
///* ********************************************************************* */
//TEST ( SQPOptimizer, inequality_avoid ) {
//	// create the graph
//	NLGraph graph; Config2D feasible;
//	boost::tie(graph, feasible) = obstacleAvoidGraph();
//
//	// create the rest of the config
//	shared_ptr<Config2D> init(new Config2D(feasible));
//	PoseKey x2(2);
//	init->insert(x2, Point2(5.0, 100.0));
//
//	// create an ordering
//	Ordering ord;
//	ord += "x1", "x2", "x3", "l1";
//
//	// create an optimizer
//	Optimizer optimizer(graph, ord, init);
//
//	// perform an iteration of optimization
//	// NOTE: the constraint will be inactive in the first iteration,
//	// so it will violate the constraint after one iteration
//	Optimizer afterOneIteration = optimizer.iterate(Optimizer::SILENT);
//
//	Config2D exp1(feasible);
//	exp1.insert(x2, Point2(5.0, 0.0));
//	CHECK(assert_equal(exp1, *(afterOneIteration.config())));
//
//	// the second iteration will activate the constraint and force the
//	// config to a viable configuration.
//	Optimizer after2ndIteration = afterOneIteration.iterate(Optimizer::SILENT);
//
//	Config2D exp2(feasible);
//	exp2.insert(x2, Point2(5.0, 0.5));
//	CHECK(assert_equal(exp2, *(after2ndIteration.config())));
//}

///* ********************************************************************* */
//TEST ( SQPOptimizer, inequality_avoid_iterative ) {
//	// create the graph
//	NLGraph graph; Config2D feasible;
//	boost::tie(graph, feasible) = obstacleAvoidGraph();
//
//	// create the rest of the config
//	shared_ptr<Config2D> init(new Config2D(feasible));
//	PoseKey x2(2);
//	init->insert(x2, Point2(5.0, 100.0));
//
//	// create an ordering
//	Ordering ord;
//	ord += "x1", "x2", "x3", "l1";
//
//	// create an optimizer
//	Optimizer optimizer(graph, ord, init);
//
//	double relThresh = 1e-5; // minimum change in error between iterations
//	double absThresh = 1e-5; // minimum error necessary to converge
//	double constraintThresh = 1e-9; // minimum constraint error to be feasible
//	Optimizer final = optimizer.iterateSolve(relThresh, absThresh, constraintThresh);
//
//	// verify
//	Config2D exp2(feasible);
//	exp2.insert(x2, Point2(5.0, 0.5));
//	CHECK(assert_equal(exp2, *(final.config())));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
