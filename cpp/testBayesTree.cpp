/**
 * @file    testBayesTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesChain.h"
#include "smallExample.h"
#include "BayesTree.h"

using namespace gtsam;

/* ************************************************************************* */
TEST( BayesTree, constructor )
{
	LinearFactorGraph factorGraph = createLinearFactorGraph();
  Ordering ordering;
  ordering.push_back("x2");
  ordering.push_back("l1");
  ordering.push_back("x1");
	SymbolicBayesChain symbolicBayesChain(factorGraph,ordering);
	BayesTree<SymbolicConditional> bayesTree(symbolicBayesChain);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
