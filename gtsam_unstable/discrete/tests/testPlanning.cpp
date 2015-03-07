/*
 * testPlanning.cpp
 * @brief develop code for planning example
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

//#include <gtsam/discrete/Planner.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST_UNSAFE(Planner, allInOne)
{
	// A small planning problem
	// First variable is location
	DiscreteKey location(0,3),
			haveRock(1,2), haveSoil(2,2), haveImage(3,2),
			commRock(4,2), commSoil(5,2), commImage(6,2);

	// There are 3 actions:
	// Drive, communicate, sample
}


/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

