/**
 * @file    timeLinearFactorGraph.cpp
 * @brief   Time elimination with simple Kalman Smoothing example
 * @author  Frank Dellaert
 */

#include <time.h>
#include <CppUnitLite/TestHarness.h>
#include "SmallExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Create a Kalman smoother for t=1:T and optimize
double timeKalmanSmoother(int T) {
	LinearFactorGraph smoother = createSmoother(T);
	Ordering ordering;
	for (int t = 1; t <= T; t++) ordering.push_back(symbol('x',t));
	clock_t start = clock();
	smoother.optimize(ordering);
	clock_t end = clock ();
	double dif = (double)(end - start) / CLOCKS_PER_SEC;
	return dif;
}

/* ************************************************************************* */
TEST(timeLinearFactorGraph, linearTime)
{
	double time1 = timeKalmanSmoother(1000); cout << time1 << endl;
	double time2 = timeKalmanSmoother(2000); cout << time2 << endl;
	DOUBLES_EQUAL(2*time1,time2,0.001);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */
