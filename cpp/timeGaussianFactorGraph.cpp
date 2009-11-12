/**
 * @file    timeGaussianFactorGraph.cpp
 * @brief   Time elimination with simple Kalman Smoothing example
 * @author  Frank Dellaert
 */

#include <time.h>
#include <CppUnitLite/TestHarness.h>
#include "smallExample.h"
#include "Ordering.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Create a Kalman smoother for t=1:T and optimize
double timeKalmanSmoother(int T) {
	GaussianFactorGraph smoother = createSmoother(T);
	Ordering ordering;
	for (int t = 1; t <= T; t++) ordering.push_back(symbol('x',t));
	clock_t start = clock();
	smoother.optimize(ordering);
	clock_t end = clock ();
	double dif = (double)(end - start) / CLOCKS_PER_SEC;
	return dif;
}

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, linearTime)
{
	int T = 1000;
	double time1 = timeKalmanSmoother(  T); // cout << time1 << endl;
	double time2 = timeKalmanSmoother(2*T); // cout << time2 << endl;
	DOUBLES_EQUAL(2*time1,time2,0.001);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */
