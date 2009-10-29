/**
 * @file    testSymbolicBayesChain.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "smallExample.h"
#include "FactorGraph-inl.h"
#include "BayesChain-inl.h"
#include "SymbolicBayesChain-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicBayesChain, constructor )
{
	// Create manually
	SymbolicConditional::shared_ptr x2(new SymbolicConditional("x1", "l1"));
	SymbolicConditional::shared_ptr l1(new SymbolicConditional("x1"));
	SymbolicConditional::shared_ptr x1(new SymbolicConditional());
	map<string, SymbolicConditional::shared_ptr> nodes;
	nodes.insert(make_pair("x2", x2));
	nodes.insert(make_pair("l1", l1));
	nodes.insert(make_pair("x1", x1));
	SymbolicBayesChain expected(nodes);

	// Create from a factor graph
	Ordering ordering;
	ordering.push_back("x2");
	ordering.push_back("l1");
	ordering.push_back("x1");
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicBayesChain actual(factorGraph, ordering);
	//CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
