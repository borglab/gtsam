/**
 * @file    testBayesTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesChain-inl.h"
#include "smallExample.h"
#include "BayesTree.h"

using namespace gtsam;

/* ************************************************************************* */
TEST( BayesTree, constructor )
{
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicBayesChain symbolicBayesChain(factorGraph);
	BayesTree<SymbolicConditional> bayesTree(symbolicBayesChain);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
