/**********************************************************
Written by Alireza Fathi, 2nd of April 2009
**********************************************************/

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "NonlinearFactorGraph.h"
#include "VSLAMFactor.h"

using namespace std;
using namespace gtsam;

// TODO: test VSLAMFactor !!!

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */

