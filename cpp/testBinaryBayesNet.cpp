/**
 * @file    testBinaryBayesNet.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
TEST( BinaryBayesNet, constructor )
{
	map<string,BinaryCPT> tables;
	BinaryCPT pA(0.01);tables.insert("A",pA);
	BinaryCPT pB("S",0.6,0.3);
	BinaryBayesNet binaryBayesNet(tables);
	BinaryConfig allFalse(false,false,false,...);
	DOUBLES_EQUAL(0.12,binaryBayesNet.probability(allFalse));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
