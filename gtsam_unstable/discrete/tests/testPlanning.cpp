/*
 * testPlanning.cpp
 * @brief develop code for planning example
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

#include <gtsam2/discrete/Planner.h>
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
	DiscreteKey location("location",3),
			haveRock("haveRock",2), haveSoil("haveSoil",2), haveImage("haveImage",2),
			commRock("commRock",2), commSoil("commSoil",2), commImage("commImage",2),
			haveRock("haveRock",2), haveSoil("haveSoil",2), haveImage("haveImage",2);

	// There are 3 actions:
	// Drive, communicate, sample
}


/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

