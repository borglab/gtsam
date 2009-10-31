/**
 * @file    testSymbolicBayesChain.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "smallExample.h"
#include "SymbolicBayesChain.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicBayesChain, constructor )
{
	// Create manually
	SymbolicConditional::shared_ptr
		x2(new SymbolicConditional("x1", "l1")),
		l1(new SymbolicConditional("x1")),
		x1(new SymbolicConditional());
	map<string, SymbolicConditional::shared_ptr> nodes;
	insert(nodes)("x2", x2)("l1", l1)("x1", x1);
	SymbolicBayesChain expected(nodes);

	// Create from a factor graph
	Ordering ordering;
	ordering += "x2","l1","x1";
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicBayesChain actual(factorGraph, ordering);
	CHECK(assert_equal(expected, actual));

	//bayesChain.ordering();
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
