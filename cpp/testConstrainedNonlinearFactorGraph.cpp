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

typedef boost::shared_ptr<NonlinearFactor> shared;

TEST( ConstrainedNonlinearFactorGraph, equals )
{
	ConstrainedNonlinearFactorGraph fg  = createConstrainedNonlinearFactorGraph();
	ConstrainedNonlinearFactorGraph fg2 = createConstrainedNonlinearFactorGraph();
	CHECK( fg.equals(fg2) );
}

TEST( ConstrainedNonlinearFactorGraph, copy )
{
	NonlinearFactorGraph nfg = createNonlinearFactorGraph();
	ConstrainedNonlinearFactorGraph actual(nfg);

	shared f1 = nfg[0];
	shared f2 = nfg[1];
	shared f3 = nfg[2];
	shared f4 = nfg[3];

	ConstrainedNonlinearFactorGraph expected;
	expected.push_back(f1);
	expected.push_back(f2);
	expected.push_back(f3);
	expected.push_back(f4);

	CHECK(actual.equals(expected));
}

TEST( ConstrainedNonlinearFactorGraph, baseline )
{
	// use existing examples
	NonlinearFactorGraph nfg = createNonlinearFactorGraph();
	ConstrainedNonlinearFactorGraph cfg(nfg);

	FGConfig initial = createNoisyConfig();
	ConstrainedLinearFactorGraph linearized = cfg.linearize(initial);
	LinearFactorGraph lfg = createLinearFactorGraph();
	ConstrainedLinearFactorGraph expected(lfg);

	CHECK(expected.equals(linearized));
}

TEST( ConstrainedNonlinearFactorGraph, convert )
{
	NonlinearFactorGraph expected = createNonlinearFactorGraph();
	ConstrainedNonlinearFactorGraph cfg(expected);
	NonlinearFactorGraph actual = cfg.convert();
	CHECK(actual.equals(expected));
}

TEST( ConstrainedNonlinearFactorGraph, linearize_and_solve )
{
	ConstrainedNonlinearFactorGraph nfg = createConstrainedNonlinearFactorGraph();
	FGConfig lin = createConstrainedLinConfig();
	ConstrainedLinearFactorGraph actual_lfg = nfg.linearize(lin);
	FGConfig actual = actual_lfg.optimize(actual_lfg.getOrdering());

	FGConfig expected = createConstrainedCorrectDelta();
	CHECK(actual.equals(expected));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

