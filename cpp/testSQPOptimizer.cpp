/*
 * @file testSQPOptimizer.cpp
 * @brief tests the optimization algorithm for nonlinear graphs with nonlinear constraints
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/map.hpp> // for insert
#include <boost/bind.hpp>

#define GTSAM_MAGIC_KEY

#include <simulated2D.h>
#include "NonlinearFactorGraph.h"
#include "NonlinearConstraint.h"
#include "NonlinearEquality.h"
#include "VectorConfig.h"
#include "Ordering.h"
#include "NonlinearOptimizer.h"
#include "SQPOptimizer.h"

// implementations
#include "NonlinearConstraint-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "SQPOptimizer-inl.h"

using namespace std;
using namespace gtsam;
using namespace boost;
using namespace boost::assign;
using namespace simulated2D;
	
static sharedGaussian sigma(noiseModel::Isotropic::Sigma(1,0.1));

// typedefs
typedef simulated2D::Config Config2D;
typedef boost::shared_ptr<Config2D> shared_config;
typedef NonlinearFactorGraph<Config2D> NLGraph;
typedef boost::shared_ptr<NonlinearFactor<Config2D> > shared;

namespace map_warp_example {
typedef NonlinearConstraint1<
	Config2D, simulated2D::PoseKey, Point2> NLC1;
typedef NonlinearConstraint2<
	Config2D, simulated2D::PointKey, Point2, simulated2D::PointKey, Point2> NLC2;
} // \namespace map_warp_example


/* ********************************************************************* */
TEST ( SQPOptimizer, basic ) {
	// create a basic optimizer
	NLGraph graph;
	Ordering ordering;
	shared_config config(new Config2D);

	SQPOptimizer<NLGraph, Config2D> optimizer(graph, ordering, config);

	// verify components
	CHECK(assert_equal(graph, *(optimizer.graph())));
	CHECK(assert_equal(ordering, *(optimizer.ordering())));
	CHECK(assert_equal(*config, *(optimizer.config())));
}

/* ********************************************************************* */
// Example that moves two separate maps into the same frame of reference
// Note that this is a linear example, so it should converge in one step
/* ********************************************************************* */

namespace sqp_LinearMapWarp2 {
// binary constraint between landmarks
/** g(x) = x-y = 0 */
Vector g_func(const Config2D& config, const PointKey& key1, const PointKey& key2) {
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
Vector g_func(const Config2D& config, const PoseKey& key) {
	Point2 p = config[key]-Point2(1.0, 1.0);
	return Vector_(2, p.x(), p.y());
}

/** jacobian at x1 */
Matrix jac_g(const Config2D& config) {
	return eye(2);
}
} // \namespace sqp_LinearMapWarp12

typedef SQPOptimizer<NLGraph, Config2D> Optimizer;

/**
 * Creates the graph with each robot seeing the landmark, and it is
 * known that it is the same landmark
 */
NLGraph linearMapWarpGraph() {
	using namespace map_warp_example;
	// keys
	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);

	// constant constraint on x1
	list<Symbol> keyx; keyx += "x1";
	shared_ptr<NLC1> c1(new NLC1(boost::bind(sqp_LinearMapWarp1::g_func, _1, x1),
							x1, boost::bind(sqp_LinearMapWarp1::jac_g, _1),
							2, "L1"));

	// measurement from x1 to l1
	Point2 z1(0.0, 5.0);
	shared f1(new simulated2D::Measurement(z1, sigma, 1,1));

	// measurement from x2 to l2
	Point2 z2(-4.0, 0.0);
	shared f2(new simulated2D::Measurement(z2, sigma, 2,2));

	// equality constraint between l1 and l2
	list<Symbol> keys; keys += "l1", "l2";
	shared_ptr<NLC2> c2 (new NLC2(
			boost::bind(sqp_LinearMapWarp2::g_func, _1, l1, l2),
			l1, boost::bind(sqp_LinearMapWarp2::jac_g1, _1),
			l2, boost::bind(sqp_LinearMapWarp2::jac_g2, _1),
			2, "L12"));

	// construct the graph
	NLGraph graph;
	graph.push_back(c1);
	graph.push_back(c2);
	graph.push_back(f1);
	graph.push_back(f2);

	return graph;
}

/* ********************************************************************* */
TEST ( SQPOptimizer, map_warp_initLam ) {
	bool verbose = false;
	// get a graph
	NLGraph graph = linearMapWarpGraph();

	// keys
	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);

	// create an initial estimate
	shared_config initialEstimate(new Config2D);
	initialEstimate->insert(x1, Point2(1.0, 1.0));
	initialEstimate->insert(l1, Point2(1.0, 6.0));
	initialEstimate->insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert(x2, Point2(0.0, 0.0)); // other pose starts at origin

	// create an initial estimate for the lagrange multiplier
	shared_ptr<VectorConfig> initLagrange(new VectorConfig);
	initLagrange->insert("L12", Vector_(2, 1.0, 1.0));
	initLagrange->insert("L1", Vector_(2, 1.0, 1.0));

	// create an ordering
	Ordering ordering;
	ordering += "x1", "x2", "l1", "l2", "L12", "L1";

	// create an optimizer
	Optimizer optimizer(graph, ordering, initialEstimate, initLagrange);
	if (verbose) optimizer.print("Initialized Optimizer");

	// perform an iteration of optimization
	Optimizer oneIteration = optimizer.iterate(Optimizer::SILENT);

	// get the config back out and verify
	Config2D actual = *(oneIteration.config());
	Config2D expected;
	expected.insert(x1, Point2(1.0, 1.0));
	expected.insert(l1, Point2(1.0, 6.0));
	expected.insert(l2, Point2(1.0, 6.0));
	expected.insert(x2, Point2(5.0, 6.0));
	CHECK(assert_equal(expected, actual));
}

/* ********************************************************************* */
TEST ( SQPOptimizer, map_warp ) {
	bool verbose = false;
	// get a graph
	NLGraph graph = linearMapWarpGraph();
	if (verbose) graph.print("Initial map warp graph");

	// keys
	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);

	// create an initial estimate
	shared_config initialEstimate(new Config2D);
	initialEstimate->insert(x1, Point2(1.0, 1.0));
	initialEstimate->insert(l1, Point2(.0, 6.0));
	initialEstimate->insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert(x2, Point2(0.0, 0.0)); // other pose starts at origin

	// create an ordering
	Ordering ordering;
	ordering += "x1", "x2", "l1", "l2";

	// create an optimizer
	Optimizer optimizer(graph, ordering, initialEstimate);

	// perform an iteration of optimization
	Optimizer oneIteration = optimizer.iterate(Optimizer::SILENT);

	// get the config back out and verify
	Config2D actual = *(oneIteration.config());
	Config2D expected;
	expected.insert(x1, Point2(1.0, 1.0));
	expected.insert(l1, Point2(1.0, 6.0));
	expected.insert(l2, Point2(1.0, 6.0));
	expected.insert(x2, Point2(5.0, 6.0));
	CHECK(assert_equal(expected, actual));
}

/* ********************************************************************* */
// This is an obstacle avoidance demo, where there is a trajectory of
// three points, where there is a circular obstacle in the middle.  There
// is a binary inequality constraint connecting the obstacle to the
// states, which enforces a minimum distance.
/* ********************************************************************* */

typedef NonlinearConstraint2<Config2D, PoseKey, Point2, PointKey, Point2> AvoidConstraint;
typedef shared_ptr<AvoidConstraint> shared_a;
typedef NonlinearEquality<Config2D, simulated2D::PoseKey, Point2> PoseConstraint;
typedef shared_ptr<PoseConstraint> shared_pc;
typedef NonlinearEquality<Config2D, simulated2D::PointKey, Point2> ObstacleConstraint;
typedef shared_ptr<ObstacleConstraint> shared_oc;


namespace sqp_avoid1 {
// avoidance radius
double radius = 1.0;

// binary avoidance constraint
/** g(x) = ||x2-obs||^2 - radius^2 > 0 */
Vector g_func(const Config2D& config, const PoseKey& x, const PointKey& obs) {
	double dist2 = config[x].dist(config[obs]);
	double thresh = radius*radius;
	return Vector_(1, dist2-thresh);
}

/** jacobian at pose */
Matrix jac_g1(const Config2D& config, const PoseKey& x, const PointKey& obs) {
	Point2 p = config[x]-config[obs];
	return Matrix_(1,2, 2.0*p.x(), 2.0*p.y());
}

/** jacobian at obstacle */
Matrix jac_g2(const Config2D& config, const PoseKey& x, const PointKey& obs) {
	Point2 p = config[x]-config[obs];
	return Matrix_(1,2, -2.0*p.x(), -2.0*p.y());
}
}

pair<NLGraph, Config2D> obstacleAvoidGraph() {
	// Keys
	PoseKey x1(1), x2(2), x3(3);
	PointKey l1(1);

	// Constrained Points
	Point2 pt_x1,
		   pt_x3(10.0, 0.0),
		   pt_l1(5.0, -0.5);

	shared_pc e1(new PoseConstraint(x1, pt_x1));
	shared_pc e2(new PoseConstraint(x3, pt_x3));
	shared_oc e3(new ObstacleConstraint(l1, pt_l1));

	// measurement from x1 to x2
	Point2 x1x2(5.0, 0.0);
	shared f1(new simulated2D::Odometry(x1x2, sigma, 1,2));

	// measurement from x2 to x3
	Point2 x2x3(5.0, 0.0);
	shared f2(new simulated2D::Odometry(x2x3, sigma, 2,3));

	// create a binary inequality constraint that forces the middle point away from
	//  the obstacle
	shared_a c1(new AvoidConstraint(boost::bind(sqp_avoid1::g_func, _1, x2, l1),
							x2, boost::bind(sqp_avoid1::jac_g1, _1, x2, l1),
						    l1,boost::bind(sqp_avoid1::jac_g2, _1, x2, l1),
						    1, "L20", false));

	// construct the graph
	NLGraph graph;
	graph.push_back(e1);
	graph.push_back(e2);
	graph.push_back(e3);
	graph.push_back(c1);
	graph.push_back(f1);
	graph.push_back(f2);

	// make a config of the fixed values, for convenience
	Config2D config;
	config.insert(x1, pt_x1);
	config.insert(x3, pt_x3);
	config.insert(l1, pt_l1);

	return make_pair(graph, config);
}

/* ********************************************************************* */
TEST ( SQPOptimizer, inequality_avoid ) {
	// create the graph
	NLGraph graph; Config2D feasible;
	boost::tie(graph, feasible) = obstacleAvoidGraph();

	// create the rest of the config
	shared_ptr<Config2D> init(new Config2D(feasible));
	PoseKey x2(2);
	init->insert(x2, Point2(5.0, 100.0));

	// create an ordering
	Ordering ord;
	ord += "x1", "x2", "x3", "l1";

	// create an optimizer
	Optimizer optimizer(graph, ord, init);

	// perform an iteration of optimization
	// NOTE: the constraint will be inactive in the first iteration,
	// so it will violate the constraint after one iteration
	Optimizer afterOneIteration = optimizer.iterate(Optimizer::SILENT);

	Config2D exp1(feasible);
	exp1.insert(x2, Point2(5.0, 0.0));
	CHECK(assert_equal(exp1, *(afterOneIteration.config())));

	// the second iteration will activate the constraint and force the
	// config to a viable configuration.
	Optimizer after2ndIteration = afterOneIteration.iterate(Optimizer::SILENT);

	Config2D exp2(feasible);
	exp2.insert(x2, Point2(5.0, 0.5));
	CHECK(assert_equal(exp2, *(after2ndIteration.config())));
}

/* ********************************************************************* */
TEST ( SQPOptimizer, inequality_avoid_iterative ) {
	// create the graph
	NLGraph graph; Config2D feasible;
	boost::tie(graph, feasible) = obstacleAvoidGraph();

	// create the rest of the config
	shared_ptr<Config2D> init(new Config2D(feasible));
	PoseKey x2(2);
	init->insert(x2, Point2(5.0, 100.0));

	// create an ordering
	Ordering ord;
	ord += "x1", "x2", "x3", "l1";

	// create an optimizer
	Optimizer optimizer(graph, ord, init);

	double relThresh = 1e-5; // minimum change in error between iterations
	double absThresh = 1e-5; // minimum error necessary to converge
	double constraintThresh = 1e-9; // minimum constraint error to be feasible
	Optimizer final = optimizer.iterateSolve(relThresh, absThresh, constraintThresh);

	// verify
	Config2D exp2(feasible);
	exp2.insert(x2, Point2(5.0, 0.5));
	CHECK(assert_equal(exp2, *(final.config()))); // FAILS
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
