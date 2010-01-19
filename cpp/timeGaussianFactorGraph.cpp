/**
 * @file    timeGaussianFactorGraph.cpp
 * @brief   Time elimination with simple Kalman Smoothing example
 * @author  Frank Dellaert
 */

#define GTSAM_MAGIC_KEY

#include <time.h>
#include <CppUnitLite/TestHarness.h>
#include "smallExample.h"
#include "Ordering.h"

using namespace std;
using namespace gtsam;
using namespace example;

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
// Create a planar factor graph and optimize
double timePlanarSmoother(int N) {
	GaussianFactorGraph fg;
	VectorConfig config;
	boost::tie(fg,config) = planarGraph(N);
	Ordering ordering = fg.getOrdering();
	clock_t start = clock();
	fg.optimize(ordering);
	clock_t end = clock ();
	double dif = (double)(end - start) / CLOCKS_PER_SEC;
	return dif;
}

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, linearTime)
{
	int T = 1000;
	double time1 = timeKalmanSmoother(  T);  cout << time1 << endl;
	double time2 = timeKalmanSmoother(2*T);  cout << time2 << endl;
	DOUBLES_EQUAL(2*time1,time2,0.1);
}

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, planar)
{
	// 1741: 8.12, 8.12, 8.12, 8.14, 8.16
	// 1742: 5.97, 5.97, 5.97, 5.99, 6.02
	// 1746: 5.96, 5.96, 5.97, 6.00, 6.04
	// 1748: 5.91, 5.92, 5.93, 5.95, 5.96
	// 1839: 0.206956 0.206939 0.206213 0.206092 0.206780 // colamd !!!!
	int N = 30;
	double time = timePlanarSmoother(N); cout << time << endl;
	DOUBLES_EQUAL(5.97,time,0.1);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */
