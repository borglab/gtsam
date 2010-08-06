/**
 * @file testNonlinearEqualityConstraint.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <simulated2DConstraints.h>
#include <NonlinearFactorGraph-inl.h>
#include <NonlinearOptimizer-inl.h>

using namespace std;
using namespace gtsam;

static const double tol = 1e-9;

SharedDiagonal hard_model = noiseModel::Constrained::All(2);
SharedDiagonal soft_model = noiseModel::Isotropic::Sigma(2, 1.0);

typedef NonlinearFactorGraph<simulated2D::Config> Graph;
typedef boost::shared_ptr<Graph> shared_graph;
typedef boost::shared_ptr<simulated2D::Config> shared_config;
typedef NonlinearOptimizer<Graph, simulated2D::Config> Optimizer;

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, unary_basics ) {
	Point2 pt(1.0, 2.0);
	simulated2D::PoseKey key(1);
	double mu = 1000.0;
	simulated2D::UnaryEqualityConstraint constraint(pt, key, mu);

	simulated2D::Config config1;
	config1.insert(key, pt);
	EXPECT(constraint.active(config1));
	EXPECT(assert_equal(zero(2), constraint.evaluateError(pt), tol));
	EXPECT(assert_equal(zero(2), constraint.unwhitenedError(config1), tol));
	EXPECT_DOUBLES_EQUAL(0.0, constraint.error(config1), tol);

	simulated2D::Config config2;
	Point2 ptBad1(2.0, 2.0);
	config2.insert(key, ptBad1);
	EXPECT(constraint.active(config2));
	EXPECT(assert_equal(Vector_(2, 1.0, 0.0), constraint.evaluateError(ptBad1), tol));
	EXPECT(assert_equal(Vector_(2, 1.0, 0.0), constraint.unwhitenedError(config2), tol));
	EXPECT_DOUBLES_EQUAL(1000.0, constraint.error(config2), tol);
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, unary_linearization ) {
	Point2 pt(1.0, 2.0);
	simulated2D::PoseKey key(1);
	double mu = 1000.0;
	simulated2D::UnaryEqualityConstraint constraint(pt, key, mu);

	simulated2D::Config config1;
	config1.insert(key, pt);
	GaussianFactor::shared_ptr actual1 = constraint.linearize(config1);
	GaussianFactor::shared_ptr expected1(new GaussianFactor(key, eye(2,2), zero(2), hard_model));
	EXPECT(assert_equal(*expected1, *actual1, tol));

	simulated2D::Config config2;
	Point2 ptBad(2.0, 2.0);
	config2.insert(key, ptBad);
	GaussianFactor::shared_ptr actual2 = constraint.linearize(config2);
	GaussianFactor::shared_ptr expected2(new GaussianFactor(key, eye(2,2), Vector_(2,-1.0,0.0), hard_model));
	EXPECT(assert_equal(*expected2, *actual2, tol));
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, unary_simple_optimization ) {
	// create a single-node graph with a soft and hard constraint to
	// ensure that the hard constraint overrides the soft constraint
	Point2 truth_pt(1.0, 2.0);
	simulated2D::PoseKey key(1);
	double mu = 1000.0;
	simulated2D::UnaryEqualityConstraint::shared_ptr constraint(
			new simulated2D::UnaryEqualityConstraint(truth_pt, key, mu));

	Point2 badPt(100.0, -200.0);
	simulated2D::Prior::shared_ptr factor(
			new simulated2D::Prior(badPt, soft_model, key));

	shared_graph graph(new Graph());
	graph->push_back(constraint);
	graph->push_back(factor);

	shared_config initConfig(new simulated2D::Config());
	initConfig->insert(key, badPt);

	Optimizer::shared_config actual = Optimizer::optimizeLM(graph, initConfig);
	simulated2D::Config expected;
	expected.insert(key, truth_pt);
	CHECK(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, odo_basics ) {
	Point2 x1(1.0, 2.0), x2(2.0, 3.0), odom(1.0, 1.0);
	simulated2D::PoseKey key1(1), key2(2);
	double mu = 1000.0;
	simulated2D::OdoEqualityConstraint constraint(odom, key1, key2, mu);

	simulated2D::Config config1;
	config1.insert(key1, x1);
	config1.insert(key2, x2);
	EXPECT(constraint.active(config1));
	EXPECT(assert_equal(zero(2), constraint.evaluateError(x1, x2), tol));
	EXPECT(assert_equal(zero(2), constraint.unwhitenedError(config1), tol));
	EXPECT_DOUBLES_EQUAL(0.0, constraint.error(config1), tol);

	simulated2D::Config config2;
	Point2 x1bad(2.0, 2.0);
	Point2 x2bad(2.0, 2.0);
	config2.insert(key1, x1bad);
	config2.insert(key2, x2bad);
	EXPECT(constraint.active(config2));
	EXPECT(assert_equal(Vector_(2, -1.0, -1.0), constraint.evaluateError(x1bad, x2bad), tol));
	EXPECT(assert_equal(Vector_(2, -1.0, -1.0), constraint.unwhitenedError(config2), tol));
	EXPECT_DOUBLES_EQUAL(2000.0, constraint.error(config2), tol);
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, odo_linearization ) {
	Point2 x1(1.0, 2.0), x2(2.0, 3.0), odom(1.0, 1.0);
	simulated2D::PoseKey key1(1), key2(2);
	double mu = 1000.0;
	simulated2D::OdoEqualityConstraint constraint(odom, key1, key2, mu);

	simulated2D::Config config1;
	config1.insert(key1, x1);
	config1.insert(key2, x2);
	GaussianFactor::shared_ptr actual1 = constraint.linearize(config1);
	GaussianFactor::shared_ptr expected1(
			new GaussianFactor(key1, -eye(2,2), key2, eye(2,2), zero(2), hard_model));
	EXPECT(assert_equal(*expected1, *actual1, tol));

	simulated2D::Config config2;
	Point2 x1bad(2.0, 2.0);
	Point2 x2bad(2.0, 2.0);
	config2.insert(key1, x1bad);
	config2.insert(key2, x2bad);
	GaussianFactor::shared_ptr actual2 = constraint.linearize(config2);
	GaussianFactor::shared_ptr expected2(
			new GaussianFactor(key1, -eye(2,2), key2, eye(2,2), Vector_(2, 1.0, 1.0), hard_model));
	EXPECT(assert_equal(*expected2, *actual2, tol));
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, odo_simple_optimize ) {
	// create a two-node graph, connected by an odometry constraint, with
	// a hard prior on one variable, and a conflicting soft prior
	// on the other variable - the constraints should override the soft constraint
	Point2 truth_pt1(1.0, 2.0), truth_pt2(3.0, 2.0);
	simulated2D::PoseKey key1(1), key2(2);

	// hard prior on x1
	simulated2D::UnaryEqualityConstraint::shared_ptr constraint1(
			new simulated2D::UnaryEqualityConstraint(truth_pt1, key1));

	// soft prior on x2
	Point2 badPt(100.0, -200.0);
	simulated2D::Prior::shared_ptr factor(
			new simulated2D::Prior(badPt, soft_model, key2));

	// odometry constraint
	simulated2D::OdoEqualityConstraint::shared_ptr constraint2(
			new simulated2D::OdoEqualityConstraint(
					gtsam::between(truth_pt1, truth_pt2), key1, key2));

	shared_graph graph(new Graph());
	graph->push_back(constraint1);
	graph->push_back(constraint2);
	graph->push_back(factor);

	shared_config initConfig(new simulated2D::Config());
	initConfig->insert(key1, Point2());
	initConfig->insert(key2, badPt);

	Optimizer::shared_config actual = Optimizer::optimizeLM(graph, initConfig);
	simulated2D::Config expected;
	expected.insert(key1, truth_pt1);
	expected.insert(key2, truth_pt2);
	CHECK(assert_equal(expected, *actual, tol));

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


