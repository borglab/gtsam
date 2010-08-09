/**
 * @file testNonlinearInequalityConstraint.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <simulated2DConstraints.h>
#include <NonlinearFactorGraph-inl.h>
#include <NonlinearOptimizer-inl.h>

namespace iq2D = gtsam::simulated2D::inequality_constraints;
using namespace std;
using namespace gtsam;

static const double tol = 1e-5;

SharedDiagonal soft_model2 = noiseModel::Unit::Create(2);
SharedDiagonal hard_model1 = noiseModel::Constrained::All(1);

typedef NonlinearFactorGraph<simulated2D::Config> Graph;
typedef boost::shared_ptr<Graph> shared_graph;
typedef boost::shared_ptr<simulated2D::Config> shared_config;
typedef NonlinearOptimizer<Graph, simulated2D::Config> Optimizer;

// some simple inequality constraints
simulated2D::PoseKey key(1);
double mu = 10.0;
iq2D::PoseXInequality constraint1(key, 1.0, mu);
iq2D::PoseYInequality constraint2(key, 2.0, mu);

/* ************************************************************************* */
TEST( testNonlinearInequalityConstraint, unary_basics_inactive ) {
	Point2 pt1(2.0, 3.0);
	simulated2D::Config config1;
	config1.insert(key, pt1);
	EXPECT(!constraint1.active(config1));
	EXPECT(!constraint2.active(config1));
	EXPECT(assert_equal(ones(1), constraint1.evaluateError(pt1), tol));
	EXPECT(assert_equal(ones(1), constraint2.evaluateError(pt1), tol));
	EXPECT(assert_equal(zero(1), constraint1.unwhitenedError(config1), tol));
	EXPECT(assert_equal(zero(1), constraint2.unwhitenedError(config1), tol));
	EXPECT_DOUBLES_EQUAL(0.0, constraint1.error(config1), tol);
	EXPECT_DOUBLES_EQUAL(0.0, constraint2.error(config1), tol);
}

/* ************************************************************************* */
TEST( testNonlinearInequalityConstraint, unary_basics_active ) {
	Point2 pt2(-2.0, -3.0);
	simulated2D::Config config2;
	config2.insert(key, pt2);
	EXPECT(constraint1.active(config2));
	EXPECT(constraint2.active(config2));
	EXPECT(assert_equal(repeat(1, -3.0), constraint1.evaluateError(pt2), tol));
	EXPECT(assert_equal(repeat(1, -5.0), constraint2.evaluateError(pt2), tol));
	EXPECT(assert_equal(repeat(1, -3.0), constraint1.unwhitenedError(config2), tol));
	EXPECT(assert_equal(repeat(1, -5.0), constraint2.unwhitenedError(config2), tol));
	EXPECT_DOUBLES_EQUAL(90.0, constraint1.error(config2), tol);
	EXPECT_DOUBLES_EQUAL(250.0, constraint2.error(config2), tol);
}

/* ************************************************************************* */
TEST( testNonlinearInequalityConstraint, unary_linearization_inactive) {
	Point2 pt1(2.0, 3.0);
	simulated2D::Config config1;
	config1.insert(key, pt1);
	GaussianFactor::shared_ptr actual1 = constraint1.linearize(config1);
	GaussianFactor::shared_ptr actual2 = constraint2.linearize(config1);
	EXPECT(!actual1);
	EXPECT(!actual2);
}

/* ************************************************************************* */
TEST( testNonlinearInequalityConstraint, unary_linearization_active) {
	Point2 pt2(-2.0, -3.0);
	simulated2D::Config config2;
	config2.insert(key, pt2);
	GaussianFactor::shared_ptr actual1 = constraint1.linearize(config2);
	GaussianFactor::shared_ptr actual2 = constraint2.linearize(config2);
	GaussianFactor expected1(key, Matrix_(1, 2, 1.0, 0.0), repeat(1, 3.0), hard_model1);
	GaussianFactor expected2(key, Matrix_(1, 2, 0.0, 1.0), repeat(1, 5.0), hard_model1);
	EXPECT(assert_equal(expected1, *actual1, tol));
	EXPECT(assert_equal(expected2, *actual2, tol));
}

/* ************************************************************************* */
TEST( testNonlinearInequalityConstraint, unary_simple_optimization) {
	// create a single-node graph with a soft and hard constraint to
	// ensure that the hard constraint overrides the soft constraint
	Point2 goal_pt(1.0, 2.0);
	Point2 start_pt(0.0, 1.0);

	shared_graph graph(new Graph());
	simulated2D::PoseKey x1(1);
	graph->add(iq2D::PoseXInequality(x1, 1.0));
	graph->add(iq2D::PoseYInequality(x1, 2.0));
	graph->add(simulated2D::Prior(start_pt, soft_model2, x1));

	shared_config initConfig(new simulated2D::Config());
	initConfig->insert(x1, start_pt);

	Optimizer::shared_config actual = Optimizer::optimizeLM(graph, initConfig, Optimizer::SILENT);
	simulated2D::Config expected;
	expected.insert(x1, goal_pt);
	CHECK(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


