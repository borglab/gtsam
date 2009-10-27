/**
 * @file    testSymbolicBayesChain.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "smallExample.h"
#include "SymbolicBayesChain-inl.h"

using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicBayesChain, constructor )
{
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicBayesChain symbolicChordalBayesNet(factorGraph);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
