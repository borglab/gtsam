/*
 * @file testSQPOptimizer.cpp
 * @brief tests the optimization algorithm for nonlinear graphs with nonlinear constraints
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/map.hpp> // for insert
#include <boost/bind.hpp>
#include <Simulated2DMeasurement.h>
#include <Simulated2DOdometry.h>
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
using namespace boost::assign;
	
// typedefs
typedef boost::shared_ptr<VectorConfig> shared_config;
typedef NonlinearFactorGraph<VectorConfig> NLGraph;
typedef boost::shared_ptr<NonlinearFactor<VectorConfig> > shared;
typedef boost::shared_ptr<NonlinearConstraint<VectorConfig> > shared_c;

TEST ( SQPOptimizer, basic ) {
	// create a basic optimizer
	NLGraph graph;
	Ordering ordering;
	shared_config config(new VectorConfig);

	SQPOptimizer<NLGraph, VectorConfig> optimizer(graph, ordering, config);

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
Vector g_func(const VectorConfig& config, const list<string>& keys) {
	return config[keys.front()]-config[keys.back()];
}

/** jacobian at l1 */
Matrix jac_g1(const VectorConfig& config, const list<string>& keys) {
	return eye(2);
}

/** jacobian at l2 */
Matrix jac_g2(const VectorConfig& config, const list<string>& keys) {
	return -1*eye(2);
}
} // \namespace sqp_LinearMapWarp2

namespace sqp_LinearMapWarp1 {
// Unary Constraint on x1
/** g(x) = x -[1;1] = 0 */
Vector g_func(const VectorConfig& config, const list<string>& keys) {
	return config[keys.front()]-Vector_(2, 1.0, 1.0);
}

/** jacobian at x1 */
Matrix jac_g(const VectorConfig& config, const list<string>& keys) {
	return eye(2);
}
} // \namespace sqp_LinearMapWarp12

typedef SQPOptimizer<NLGraph, VectorConfig> Optimizer;

/**
 * Creates the graph with each robot seeing the landmark, and it is
 * known that it is the same landmark
 */
NLGraph linearMapWarpGraph() {
	// constant constraint on x1
	list<string> keyx; keyx += "x1";
	boost::shared_ptr<NonlinearConstraint1<VectorConfig> > c1(
			new NonlinearConstraint1<VectorConfig>(
					boost::bind(sqp_LinearMapWarp1::g_func, _1, keyx),
					"x1", boost::bind(sqp_LinearMapWarp1::jac_g, _1, keyx),
					 2, "L_x1"));

	// measurement from x1 to l1
	Vector z1 = Vector_(2, 0.0, 5.0);
	double sigma1 = 0.1;
	shared f1(new Simulated2DMeasurement(z1, sigma1, "x1", "l1"));

	// measurement from x2 to l2
	Vector z2 = Vector_(2, -4.0, 0.0);
	double sigma2 = 0.1;
	shared f2(new Simulated2DMeasurement(z2, sigma2, "x2", "l2"));

	// equality constraint between l1 and l2
	list<string> keys; keys += "l1", "l2";
	boost::shared_ptr<NonlinearConstraint2<VectorConfig> > c2(
			new NonlinearConstraint2<VectorConfig>(
					boost::bind(sqp_LinearMapWarp2::g_func, _1, keys),
					"l1", boost::bind(sqp_LinearMapWarp2::jac_g1, _1, keys),
					"l2", boost::bind(sqp_LinearMapWarp2::jac_g2, _1, keys),
					 2, "L_l1l2"));

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

	// create an initial estimate
	shared_config initialEstimate(new VectorConfig);
	initialEstimate->insert("x1", Vector_(2, 1.0, 1.0));
	initialEstimate->insert("l1", Vector_(2, 1.0, 6.0));
	initialEstimate->insert("l2", Vector_(2, -4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert("x2", Vector_(2, 0.0, 0.0)); // other pose starts at origin

	// create an initial estimate for the lagrange multiplier
	shared_config initLagrange(new VectorConfig);
	initLagrange->insert("L_l1l2", Vector_(2, 1.0, 1.0));
	initLagrange->insert("L_x1", Vector_(2, 1.0, 1.0));

	// create an ordering
	Ordering ordering;
	ordering += "x1", "x2", "l1", "l2", "L_l1l2", "L_x1";

	// create an optimizer
	Optimizer optimizer(graph, ordering, initialEstimate, initLagrange);
	if (verbose) optimizer.print("Initialized Optimizer");

	// perform an iteration of optimization
	Optimizer oneIteration = optimizer.iterate(Optimizer::SILENT);

	// get the config back out and verify
	VectorConfig actual = *(oneIteration.config());
	VectorConfig expected;
	expected.insert("x1", Vector_(2, 1.0, 1.0));
	expected.insert("l1", Vector_(2, 1.0, 6.0));
	expected.insert("l2", Vector_(2, 1.0, 6.0));
	expected.insert("x2", Vector_(2, 5.0, 6.0));
	CHECK(assert_equal(expected, actual));
}

/* ********************************************************************* */
TEST ( SQPOptimizer, map_warp ) {
	bool verbose = false;
	// get a graph
	NLGraph graph = linearMapWarpGraph();
	if (verbose) graph.print("Initial map warp graph");

	// create an initial estimate
	shared_config initialEstimate(new VectorConfig);
	initialEstimate->insert("x1", Vector_(2, 1.0, 1.0));
	initialEstimate->insert("l1", Vector_(2, 1.0, 6.0));
	initialEstimate->insert("l2", Vector_(2, -4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert("x2", Vector_(2, 0.0, 0.0)); // other pose starts at origin

	// create an ordering
	Ordering ordering;
	ordering += "x1", "x2", "l1", "l2";

	// create an optimizer
	Optimizer optimizer(graph, ordering, initialEstimate);

	// perform an iteration of optimization
	Optimizer oneIteration = optimizer.iterate(Optimizer::SILENT);

	// get the config back out and verify
	VectorConfig actual = *(oneIteration.config());
	VectorConfig expected;
	expected.insert("x1", Vector_(2, 1.0, 1.0));
	expected.insert("l1", Vector_(2, 1.0, 6.0));
	expected.insert("l2", Vector_(2, 1.0, 6.0));
	expected.insert("x2", Vector_(2, 5.0, 6.0));
	CHECK(assert_equal(expected, actual));
}

/* ********************************************************************* */
// This is an obstacle avoidance demo, where there is a trajectory of
// three points, where there is a circular obstacle in the middle.  There
// is a binary inequality constraint connecting the obstacle to the
// states, which enforces a minimum distance.
/* ********************************************************************* */

bool vector_compare(const std::string& key,
					const VectorConfig& feasible,
					const VectorConfig& input) {
	Vector feas, lin;
	feas = feasible[key];
	lin = input[key];
	return equal_with_abs_tol(lin, feas, 1e-5);
}

typedef NonlinearConstraint1<VectorConfig> NLC1;
typedef boost::shared_ptr<NLC1> shared_NLC1;
typedef NonlinearConstraint2<VectorConfig> NLC2;
typedef boost::shared_ptr<NLC2> shared_NLC2;
typedef NonlinearEquality<VectorConfig> NLE;
typedef boost::shared_ptr<NLE> shared_NLE;

namespace sqp_avoid1 {
// avoidance radius
double radius = 1.0;

// binary avoidance constraint
/** g(x) = ||x2-obs||^2 - radius^2 > 0 */
Vector g_func(const VectorConfig& config, const list<string>& keys) {
	Vector delta = config[keys.front()]-config[keys.back()];
	double dist2 = sum(emul(delta, delta));
	double thresh = radius*radius;
	return Vector_(1, dist2-thresh);
}

/** jacobian at pose */
Matrix jac_g1(const VectorConfig& config, const list<string>& keys) {
	Vector x2 = config[keys.front()], obs = config[keys.back()];
	Vector grad = 2.0*(x2-obs);
	return Matrix_(1,2, grad(0), grad(1));
}

/** jacobian at obstacle */
Matrix jac_g2(const VectorConfig& config, const list<string>& keys) {
	Vector x2 = config[keys.front()], obs = config[keys.back()];
	Vector grad = -2.0*(x2-obs);
	return Matrix_(1,2, grad(0), grad(1));
}
}

pair<NLGraph, VectorConfig> obstacleAvoidGraph() {
	// fix start, end, obstacle positions
	VectorConfig feasible;
	feasible.insert("x1", Vector_(2, 0.0, 0.0));
	feasible.insert("x3", Vector_(2, 10.0, 0.0));
	feasible.insert("obs", Vector_(2, 5.0, -0.5));
	shared_NLE e1(new NLE("x1", feasible, 2, *vector_compare));
	shared_NLE e2(new NLE("x3", feasible, 2, *vector_compare));
	shared_NLE e3(new NLE("obs", feasible, 2, *vector_compare));

	// measurement from x1 to x2
	Vector x1x2 = Vector_(2, 5.0, 0.0);
	double sigma1 = 0.1;
	shared f1(new Simulated2DOdometry(x1x2, sigma1, "x1", "x2"));

	// measurement from x2 to x3
	Vector x2x3 = Vector_(2, 5.0, 0.0);
	double sigma2 = 0.1;
	shared f2(new Simulated2DOdometry(x2x3, sigma2, "x2", "x3"));

	// create a binary inequality constraint that forces the middle point away from
	//  the obstacle
	list<string> keys; keys += "x2", "obs";
	shared_NLC2 c1(new NLC2(boost::bind(sqp_avoid1::g_func, _1, keys),
							"x2", boost::bind(sqp_avoid1::jac_g1, _1, keys),
						    "obs",boost::bind(sqp_avoid1::jac_g2, _1, keys),
						    1, "L_x2obs", false));

	// construct the graph
	NLGraph graph;
	graph.push_back(e1);
	graph.push_back(e2);
	graph.push_back(e3);
	graph.push_back(c1);
	graph.push_back(f1);
	graph.push_back(f2);

	return make_pair(graph, feasible);
}

/* ********************************************************************* */
TEST ( SQPOptimizer, inequality_avoid ) {
	// create the graph
	NLGraph graph; VectorConfig feasible;
	boost::tie(graph, feasible) = obstacleAvoidGraph();

	// create the rest of the config
	shared_config init(new VectorConfig(feasible));
	init->insert("x2", Vector_(2, 5.0, 100.0));

	// create an ordering
	Ordering ord;
	ord += "x1", "x2", "x3", "obs";

	// create an optimizer
	Optimizer optimizer(graph, ord, init);

	// perform an iteration of optimization
	// NOTE: the constraint will be inactive in the first iteration,
	// so it will violate the constraint after one iteration
	Optimizer afterOneIteration = optimizer.iterate(Optimizer::SILENT);

	VectorConfig exp1(feasible);
	exp1.insert("x2", Vector_(2, 5.0, 0.0));
	CHECK(assert_equal(exp1, *(afterOneIteration.config())));

	// the second iteration will activate the constraint and force the
	// config to a viable configuration.
	Optimizer after2ndIteration = afterOneIteration.iterate(Optimizer::SILENT);

	VectorConfig exp2(feasible);
	exp2.insert("x2", Vector_(2, 5.0, 0.75));
	CHECK(assert_equal(exp2, *(after2ndIteration.config())));
}

/* ********************************************************************* */
TEST ( SQPOptimizer, inequality_avoid_iterative ) {
	// create the graph
	NLGraph graph; VectorConfig feasible;
	boost::tie(graph, feasible) = obstacleAvoidGraph();

	// create the rest of the config
	shared_config init(new VectorConfig(feasible));
	init->insert("x2", Vector_(2, 5.0, 100.0));

	// create an ordering
	Ordering ord;
	ord += "x1", "x2", "x3", "obs";

	// create an optimizer
	Optimizer optimizer(graph, ord, init);

	double relThresh = 1e-5; // minimum change in error between iterations
	double absThresh = 1e-5; // minimum error necessary to converge
	double constraintThresh = 1e-9; // minimum constraint error to be feasible
	Optimizer final = optimizer.iterateSolve(relThresh, absThresh, constraintThresh);

	// verify
	VectorConfig exp2(feasible);
	exp2.insert("x2", Vector_(2, 5.0, 0.75));
	CHECK(assert_equal(exp2, *(final.config())));
}

/* ********************************************************************* */
// Use boost bind to parameterize the function
namespace sqp_avoid2 {
// binary avoidance constraint
/** g(x) = ||x2-obs||^2 - radius^2 > 0 */
Vector g_func(double radius, const VectorConfig& config, const list<string>& keys) {
	Vector delta = config[keys.front()]-config[keys.back()];
	double dist2 = sum(emul(delta, delta));
	double thresh = radius*radius;
	return Vector_(1, dist2-thresh);
}

/** jacobian at pose */
Matrix jac_g1(const VectorConfig& config, const list<string>& keys) {
	Vector x2 = config[keys.front()], obs = config[keys.back()];
	Vector grad = 2.0*(x2-obs);
	return Matrix_(1,2, grad(0), grad(1));
}

/** jacobian at obstacle */
Matrix jac_g2(const VectorConfig& config, const list<string>& keys) {
	Vector x2 = config[keys.front()], obs = config[keys.back()];
	Vector grad = -2.0*(x2-obs);
	return Matrix_(1,2, grad(0), grad(1));
}
}

pair<NLGraph, VectorConfig> obstacleAvoidGraphGeneral() {
	// fix start, end, obstacle positions
	VectorConfig feasible;
	feasible.insert("x1", Vector_(2, 0.0, 0.0));
	feasible.insert("x3", Vector_(2, 10.0, 0.0));
	feasible.insert("obs", Vector_(2, 5.0, -0.5));
	shared_NLE e1(new NLE("x1", feasible, 2, *vector_compare));
	shared_NLE e2(new NLE("x3", feasible, 2, *vector_compare));
	shared_NLE e3(new NLE("obs", feasible, 2, *vector_compare));

	// measurement from x1 to x2
	Vector x1x2 = Vector_(2, 5.0, 0.0);
	double sigma1 = 0.1;
	shared f1(new Simulated2DOdometry(x1x2, sigma1, "x1", "x2"));

	// measurement from x2 to x3
	Vector x2x3 = Vector_(2, 5.0, 0.0);
	double sigma2 = 0.1;
	shared f2(new Simulated2DOdometry(x2x3, sigma2, "x2", "x3"));

	double radius = 1.0;

	// create a binary inequality constraint that forces the middle point away from
	//  the obstacle
	list<string> keys; keys += "x2", "obs";
	shared_NLC2 c1(new NLC2(boost::bind(sqp_avoid2::g_func, radius, _1, keys),
						    "x2", boost::bind(sqp_avoid2::jac_g1, _1, keys),
						    "obs", boost::bind(sqp_avoid2::jac_g2, _1, keys),
						    1, "L_x2obs", false));

	// construct the graph
	NLGraph graph;
	graph.push_back(e1);
	graph.push_back(e2);
	graph.push_back(e3);
	graph.push_back(c1);
	graph.push_back(f1);
	graph.push_back(f2);

	return make_pair(graph, feasible);
}

/* ********************************************************************* */
TEST ( SQPOptimizer, inequality_avoid_iterative_bind ) {
	// create the graph
	NLGraph graph; VectorConfig feasible;
	boost::tie(graph, feasible) = obstacleAvoidGraphGeneral();

	// create the rest of the config
	shared_config init(new VectorConfig(feasible));
	init->insert("x2", Vector_(2, 5.0, 100.0));

	// create an ordering
	Ordering ord;
	ord += "x1", "x2", "x3", "obs";

	// create an optimizer
	Optimizer optimizer(graph, ord, init);

	double relThresh = 1e-5; // minimum change in error between iterations
	double absThresh = 1e-5; // minimum error necessary to converge
	double constraintThresh = 1e-9; // minimum constraint error to be feasible
	Optimizer final = optimizer.iterateSolve(relThresh, absThresh, constraintThresh);

	// verify
	VectorConfig exp2(feasible);
	exp2.insert("x2", Vector_(2, 5.0, 0.75));
	CHECK(assert_equal(exp2, *(final.config())));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
