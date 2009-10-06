/*
 * testConstrainedNonlinearFactorGraph.cpp
 *
 *  Created on: Aug 10, 2009
 *      Author: Alex Cunningham
 */


#include <CppUnitLite/TestHarness.h>
#include "ConstrainedNonlinearFactorGraph.h"
#include "smallExample.h"

using namespace gtsam;

typedef boost::shared_ptr<NonlinearFactor<FGConfig> > shared;
typedef ConstrainedNonlinearFactorGraph<NonlinearFactor<FGConfig>,FGConfig> TestGraph;

TEST( TestGraph, equals )
{
	TestGraph fg  = createConstrainedNonlinearFactorGraph();
	TestGraph fg2 = createConstrainedNonlinearFactorGraph();
	CHECK( fg.equals(fg2) );
}

TEST( TestGraph, copy )
{
	ExampleNonlinearFactorGraph nfg = createNonlinearFactorGraph();
	TestGraph actual(nfg);

	shared f1 = nfg[0];
	shared f2 = nfg[1];
	shared f3 = nfg[2];
	shared f4 = nfg[3];

	TestGraph expected;
	expected.push_back(f1);
	expected.push_back(f2);
	expected.push_back(f3);
	expected.push_back(f4);

	CHECK(actual.equals(expected));
}

TEST( TestGraph, baseline )
{
	// use existing examples
	ExampleNonlinearFactorGraph nfg = createNonlinearFactorGraph();
	TestGraph cfg(nfg);

	FGConfig initial = createNoisyConfig();
	ConstrainedLinearFactorGraph linearized = cfg.linearize(initial);
	LinearFactorGraph lfg = createLinearFactorGraph();
	ConstrainedLinearFactorGraph expected(lfg);

	CHECK(expected.equals(linearized));
}

/*
TEST( TestGraph, convert )
{
	ExampleNonlinearFactorGraph expected = createNonlinearFactorGraph();
	TestGraph cfg(expected);
	ExampleNonlinearFactorGraph actual = cfg.convert();
	CHECK(actual.equals(expected));
}
*/

TEST( TestGraph, linearize_and_solve )
{
	TestGraph nfg = createConstrainedNonlinearFactorGraph();
	FGConfig lin = createConstrainedLinConfig();
	ConstrainedLinearFactorGraph actual_lfg = nfg.linearize(lin);
	FGConfig actual = actual_lfg.optimize(actual_lfg.getOrdering());

	FGConfig expected = createConstrainedCorrectDelta();
	CHECK(actual.equals(expected));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

