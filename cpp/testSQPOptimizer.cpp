/*
 * @file testSQPOptimizer.cpp
 * @brief tests the optimization algorithm for nonlinear graphs with nonlinear constraints
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/map.hpp> // for insert
#include <Simulated2DMeasurement.h>
#include <simulated2D.h>
#include "NonlinearFactorGraph.h"
#include "NonlinearConstraint.h"
#include "VectorConfig.h"
#include "Ordering.h"
#include "SQPOptimizer.h"

// implementations
#include "NonlinearConstraint-inl.h"
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

namespace sqp_LinearMapWarp2 {
// binary constraint between landmarks
/** g(x) = x-y = 0 */
Vector g_func(const VectorConfig& config, const std::string& key1, const std::string& key2) {
	return config[key1]-config[key2];
}

/** gradient at l1 */
Matrix grad_g1(const VectorConfig& config, const std::string& key) {
	return eye(2);
}

/** gradient at l2 */
Matrix grad_g2(const VectorConfig& config, const std::string& key) {
	return -1*eye(2);
}
} // \namespace sqp_LinearMapWarp2

namespace sqp_LinearMapWarp1 {
// Unary Constraint on x1
/** g(x) = x -[1;1] = 0 */
Vector g_func(const VectorConfig& config, const std::string& key) {
	return config[key]-Vector_(2, 1.0, 1.0);
}

/** gradient at x1 */
Matrix grad_g(const VectorConfig& config, const std::string& key) {
	return eye(2);
}
} // \namespace sqp_LinearMapWarp12

typedef SQPOptimizer<NLGraph, VectorConfig> Optimizer;

NLGraph linearMapWarpGraph() {
	// constant constraint on x1
	boost::shared_ptr<NonlinearConstraint1<VectorConfig> > c1(
			new NonlinearConstraint1<VectorConfig>(
					"x1", *sqp_LinearMapWarp1::grad_g,
					*sqp_LinearMapWarp1::g_func, 2, "L_x1"));

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
					"l1", *sqp_LinearMapWarp2::grad_g1,
					"l2", *sqp_LinearMapWarp2::grad_g2,
					*sqp_LinearMapWarp2::g_func, 2, "L_l1l2"));

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

	// perform an iteration of optimization
	Optimizer oneIteration = optimizer.iterate(Optimizer::SILENT);

	// get the config back out and verify
	VectorConfig actual = *(oneIteration.config());
	VectorConfig expected;
	expected.insert("x1", Vector_(2, 1.0, 1.0));
	expected.insert("l1", Vector_(2, 1.0, 6.0));
	expected.insert("l2", Vector_(2, 1.0, 6.0));
	expected.insert("x2", Vector_(2, 5.0, 6.0));
	CHECK(assert_equal(actual, expected));
}

/* ********************************************************************* */
TEST ( SQPOptimizer, map_warp ) {
	// get a graph
	NLGraph graph = linearMapWarpGraph();

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
	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
