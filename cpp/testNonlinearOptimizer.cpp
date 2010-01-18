/** 
 * @file    testNonlinearOptimizer.cpp
 * @brief   Unit tests for NonlinearOptimizer class
 * @author  Frank Dellaert
 */

#include <iostream>
using namespace std;

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
using namespace boost;

#define GTSAM_MAGIC_KEY

#include "Matrix.h"
#include "Ordering.h"
#include "smallExample.h"
#include "pose2SLAM.h"
#include "GaussianFactorGraph.h"
#include "NoiseModel.h"

// template definitions
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "SubgraphPreconditioner-inl.h"

using namespace gtsam;

typedef NonlinearOptimizer<ExampleNonlinearFactorGraph,VectorConfig> Optimizer;

/* ************************************************************************* */
TEST( NonlinearOptimizer, delta )
{
	shared_ptr<ExampleNonlinearFactorGraph> fg(new ExampleNonlinearFactorGraph(createNonlinearFactorGraph()));
	Optimizer::shared_config initial = sharedNoisyConfig();

	// Expected configuration is the difference between the noisy config
	// and the ground-truth config. One step only because it's linear !
	VectorConfig expected;
	Vector dl1(2);
	dl1(0) = -0.1;
	dl1(1) = 0.1;
	expected.insert("l1", dl1);
	Vector dx1(2);
	dx1(0) = -0.1;
	dx1(1) = -0.1;
	expected.insert("x1", dx1);
	Vector dx2(2);
	dx2(0) = 0.1;
	dx2(1) = -0.2;
	expected.insert("x2", dx2);

	// Check one ordering
	shared_ptr<Ordering> ord1(new Ordering());
	*ord1 += "x2","l1","x1";
	Optimizer optimizer1(fg, ord1, initial);
	VectorConfig actual1 = optimizer1.linearizeAndOptimizeForDelta();
	CHECK(assert_equal(actual1,expected));

	// Check another
	shared_ptr<Ordering> ord2(new Ordering());
	*ord2 += "x1","x2","l1";
	Optimizer optimizer2(fg, ord2, initial);
	VectorConfig actual2 = optimizer2.linearizeAndOptimizeForDelta();
	CHECK(assert_equal(actual2,expected));

	// And yet another...
	shared_ptr<Ordering> ord3(new Ordering());
	*ord3 += "l1","x1","x2";
	Optimizer optimizer3(fg, ord3, initial);
	VectorConfig actual3 = optimizer3.linearizeAndOptimizeForDelta();
	CHECK(assert_equal(actual3,expected));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, iterateLM )
{
	// really non-linear factor graph
  shared_ptr<ExampleNonlinearFactorGraph> fg(new ExampleNonlinearFactorGraph(createReallyNonlinearFactorGraph()));

	// config far from minimum
	Vector x0 = Vector_(1, 3.0);
	boost::shared_ptr<VectorConfig> config(new VectorConfig);
	config->insert("x", x0);

	// ordering
	shared_ptr<Ordering> ord(new Ordering());
	ord->push_back("x");

	// create initial optimization state, with lambda=0
	Optimizer::shared_solver solver(new Factorization<ExampleNonlinearFactorGraph, VectorConfig>);
	Optimizer optimizer(fg, ord, config, solver, 0.);

	// normal iterate
	Optimizer iterated1 = optimizer.iterate();

	// LM iterate with lambda 0 should be the same
	Optimizer iterated2 = optimizer.iterateLM();

	// Try successive iterates. TODO: ugly pointers, better way ?
	Optimizer *pointer = new Optimizer(iterated2);
	for (int i=0;i<10;i++) {
		Optimizer* newOptimizer = new Optimizer(pointer->iterateLM());
		delete pointer;
		pointer = newOptimizer;
	}
	delete(pointer);

	CHECK(assert_equal(*iterated1.config(), *iterated2.config(), 1e-9));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, optimize )
{
  shared_ptr<ExampleNonlinearFactorGraph> fg(new ExampleNonlinearFactorGraph(createReallyNonlinearFactorGraph()));

	// test error at minimum
	Vector xstar = Vector_(1, 0.0);
	VectorConfig cstar;
	cstar.insert("x", xstar);
	DOUBLES_EQUAL(0.0,fg->error(cstar),0.0);

	// test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
	Vector x0 = Vector_(1, 3.0);
	boost::shared_ptr<VectorConfig> c0(new VectorConfig);
	c0->insert("x", x0);
	DOUBLES_EQUAL(199.0,fg->error(*c0),1e-3);

	// optimize parameters
	shared_ptr<Ordering> ord(new Ordering());
	ord->push_back("x");
	double relativeThreshold = 1e-5;
	double absoluteThreshold = 1e-5;

	// initial optimization state is the same in both cases tested
	Optimizer optimizer(fg, ord, c0);

	// Gauss-Newton
	Optimizer actual1 = optimizer.gaussNewton(relativeThreshold,
			absoluteThreshold);
	CHECK(assert_equal(*(actual1.config()),cstar));

	// Levenberg-Marquardt
	Optimizer actual2 = optimizer.levenbergMarquardt(relativeThreshold,
			absoluteThreshold, Optimizer::SILENT);
	CHECK(assert_equal(*(actual2.config()),cstar));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, Factorization )
{
	typedef NonlinearOptimizer<Pose2Graph, Pose2Config, Factorization<Pose2Graph, Pose2Config> > Optimizer;

	boost::shared_ptr<Pose2Config> config(new Pose2Config);
	config->insert(1, Pose2(0.,0.,0.));
	config->insert(2, Pose2(1.5,0.,0.));

	boost::shared_ptr<Pose2Graph> graph(new Pose2Graph);
	graph->addPrior(1, Pose2(0.,0.,0.), Isotropic::Sigma(3, 1e-10));
	graph->addConstraint(1,2, Pose2(1.,0.,0.), Isotropic::Sigma(3, 1));

	boost::shared_ptr<Ordering> ordering(new Ordering);
	ordering->push_back(Pose2Config::Key(1));
	ordering->push_back(Pose2Config::Key(2));

	Optimizer optimizer(graph, ordering, config);
	Optimizer optimized = optimizer.iterateLM();

	Pose2Config expected;
	expected.insert(1, Pose2(0.,0.,0.));
	expected.insert(2, Pose2(1.,0.,0.));
	CHECK(assert_equal(expected, *optimized.config(), 1e-5));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SubgraphPCG )
{
	typedef NonlinearOptimizer<Pose2Graph, Pose2Config, SubgraphPCG<Pose2Graph, Pose2Config> > Optimizer;

	boost::shared_ptr<Pose2Config> config(new Pose2Config);
	config->insert(1, Pose2(0.,0.,0.));
	config->insert(2, Pose2(1.5,0.,0.));

	boost::shared_ptr<Pose2Graph> graph(new Pose2Graph);
	graph->addPrior(1, Pose2(0.,0.,0.), Isotropic::Sigma(3, 1e-10));
	graph->addConstraint(1,2, Pose2(1.,0.,0.), Isotropic::Sigma(3, 1));

	boost::shared_ptr<Ordering> ordering(new Ordering);
	ordering->push_back(Pose2Config::Key(1));
	ordering->push_back(Pose2Config::Key(2));

	double relativeThreshold = 1e-5;
	double absoluteThreshold = 1e-5;
	Optimizer::shared_solver solver(new SubgraphPCG<Pose2Graph, Pose2Config>(*graph, *config));
	Optimizer optimizer(graph, ordering, config, solver);
	Optimizer optimized = optimizer.gaussNewton(relativeThreshold, absoluteThreshold, Optimizer::SILENT);

	Pose2Config expected;
	expected.insert(1, Pose2(0.,0.,0.));
	expected.insert(2, Pose2(1.,0.,0.));
	CHECK(assert_equal(expected, *optimized.config(), 1e-5));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
