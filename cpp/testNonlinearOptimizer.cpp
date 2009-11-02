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

#include "Matrix.h"
#include "Ordering.h"
#include "smallExample.h"

// template definitions
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"

using namespace gtsam;

typedef NonlinearOptimizer<ExampleNonlinearFactorGraph,VectorConfig> Optimizer;

/* ************************************************************************* */
TEST( NonlinearOptimizer, delta )
{
	ExampleNonlinearFactorGraph fg = createNonlinearFactorGraph();
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
	Ordering ord1;
	ord1 += "x2","l1","x1";
	Optimizer optimizer1(fg, ord1, initial);
	VectorConfig actual1 = optimizer1.linearizeAndOptimizeForDelta();
	CHECK(assert_equal(actual1,expected));

	// Check another
	Ordering ord2;
	ord2 += "x1","x2","l1";
	Optimizer optimizer2(fg, ord2, initial);
	VectorConfig actual2 = optimizer2.linearizeAndOptimizeForDelta();
	CHECK(assert_equal(actual2,expected));

	// And yet another...
	Ordering ord3;
	ord3 += "l1","x1","x2";
	Optimizer optimizer3(fg, ord3, initial);
	VectorConfig actual3 = optimizer3.linearizeAndOptimizeForDelta();
	CHECK(assert_equal(actual3,expected));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, iterateLM )
{
	// really non-linear factor graph
	ExampleNonlinearFactorGraph fg = createReallyNonlinearFactorGraph();

	// config far from minimum
	Vector x0 = Vector_(1, 3.0);
	boost::shared_ptr<VectorConfig> config(new VectorConfig);
	config->insert("x", x0);

	// ordering
	Ordering ord;
	ord.push_back("x");

	// create initial optimization state, with lambda=0
	Optimizer optimizer(fg, ord, config, 0);

	// normal iterate
	Optimizer iterated1 = optimizer.iterate();

	// LM iterate with lambda 0 should be the same
	Optimizer iterated2 = optimizer.iterateLM();

	CHECK(assert_equal(*iterated1.config(), *iterated2.config(), 1e-9));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, optimize )
{
	ExampleNonlinearFactorGraph fg = createReallyNonlinearFactorGraph();

	// test error at minimum
	Vector xstar = Vector_(1, 0.0);
	VectorConfig cstar;
	cstar.insert("x", xstar);
	DOUBLES_EQUAL(0.0,fg.error(cstar),0.0);

	// test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
	Vector x0 = Vector_(1, 3.0);
	boost::shared_ptr<VectorConfig> c0(new VectorConfig);
	c0->insert("x", x0);
	DOUBLES_EQUAL(199.0,fg.error(*c0),1e-3);

	// optimize parameters
	Ordering ord;
	ord.push_back("x");
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
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
