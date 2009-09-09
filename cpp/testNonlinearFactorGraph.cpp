/** 
 * @file    testNonlinearFactorGraph.cpp
 * @brief   Unit tests for Non-Linear Factor Graph
 * @brief   testNonlinearFactorGraph
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

/*STL/C++*/
#include <iostream>
using namespace std;

#include <CppUnitLite/TestHarness.h>

#include "Matrix.h"
#include "smallExample.h"

using namespace gtsam;

/* ************************************************************************* */
TEST( NonlinearFactorGraph, equals )
{

	NonlinearFactorGraph fg = createNonlinearFactorGraph();
	NonlinearFactorGraph fg2 = createNonlinearFactorGraph();
	CHECK( fg.equals(fg2) );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, error )
{
	NonlinearFactorGraph fg = createNonlinearFactorGraph();

	FGConfig c1 = createConfig();
	double actual1 = fg.error(c1);
	DOUBLES_EQUAL( 0.0, actual1, 1e-9 );

	FGConfig c2 = createNoisyConfig();
	double actual2 = fg.error(c2);
	DOUBLES_EQUAL( 5.625, actual2, 1e-9 );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, probPrime )
{
	NonlinearFactorGraph fg = createNonlinearFactorGraph();
	FGConfig cfg = createConfig();

	// evaluate the probability of the factor graph
	double actual = fg.probPrime(cfg);
	double expected = 1.0;
	DOUBLES_EQUAL(expected,actual,0);
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, linearize )
{
	NonlinearFactorGraph fg = createNonlinearFactorGraph();
	FGConfig initial = createNoisyConfig();
	LinearFactorGraph linearized = fg.linearize(initial);
	LinearFactorGraph expected = createLinearFactorGraph();
	CHECK(expected.equals(linearized));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
