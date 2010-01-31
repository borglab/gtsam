/**
 * @file    timeGaussianFactorGraph.cpp
 * @brief   Time elimination with simple Kalman Smoothing example
 * @author  Frank Dellaert
 */

#define GTSAM_MAGIC_KEY

#include <time.h>
#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator += in Ordering
#include <CppUnitLite/TestHarness.h>
#include "smallExample.h"
#include "Ordering.h"

using namespace std;
using namespace gtsam;
using namespace example;
using namespace boost::assign;

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
double timePlanarSmoother(int N, bool old = true) {
	GaussianFactorGraph fg;
	VectorConfig config;
	boost::tie(fg,config) = planarGraph(N);
	Ordering ordering = fg.getOrdering();
	clock_t start = clock();
	fg.optimize(ordering, old);
	clock_t end = clock ();
	double dif = (double)(end - start) / CLOCKS_PER_SEC;
	return dif;
}

/* ************************************************************************* */
// Create a planar factor graph and eliminate
double timePlanarSmootherEliminate(int N, bool old = true) {
	GaussianFactorGraph fg;
	VectorConfig config;
	boost::tie(fg,config) = planarGraph(N);
	Ordering ordering = fg.getOrdering();
	clock_t start = clock();
	fg.eliminate(ordering, old);
	clock_t end = clock ();
	double dif = (double)(end - start) / CLOCKS_PER_SEC;
	return dif;
}

/* ************************************************************************* */
// Create a planar factor graph and join factors until matrix formation
// This variation uses the original join factors approach
double timePlanarSmootherJoinAug(int N, size_t reps) {
	GaussianFactorGraph fgBase;
	VectorConfig config;
	boost::tie(fgBase,config) = planarGraph(N);
	Ordering ordering = fgBase.getOrdering();
	Symbol key = ordering.front();

	clock_t start = clock();

	for (size_t i = 0; i<reps; ++i) {
		// setup
		GaussianFactorGraph fg(fgBase);

		// combine some factors
		GaussianFactor::shared_ptr joint_factor = removeAndCombineFactors(fg,key);

		// create an internal ordering to render Ab
		Ordering render;
		render += key;
		BOOST_FOREACH(const Symbol& k, joint_factor->keys())
		if (k != key) render += k;

		Matrix Ab = joint_factor->matrix_augmented(render,false);
	}

	clock_t end = clock ();
	double dif = (double)(end - start) / CLOCKS_PER_SEC;
	return dif;
}

/* ************************************************************************* */
// Create a planar factor graph and join factors until matrix formation
// This variation uses the single-allocate version to create the matrix
double timePlanarSmootherCombined(int N, size_t reps) {
	GaussianFactorGraph fgBase;
	VectorConfig config;
	boost::tie(fgBase,config) = planarGraph(N);
	Ordering ordering = fgBase.getOrdering();
	Symbol key = ordering.front();

	clock_t start = clock();

	for (size_t i = 0; i<reps; ++i) {
		// setup
		GaussianFactorGraph fg(fgBase);

		Ordering render; render += key; // start with variable to eliminate
		vector<GaussianFactor::shared_ptr> factors = fg.findAndRemoveFactors(key);

		set<Symbol> separator;
		Dimensions dimensions;
		BOOST_FOREACH(GaussianFactor::shared_ptr factor, factors) {
			Dimensions factor_dim = factor->dimensions();
			dimensions.insert(factor_dim.begin(), factor_dim.end());
			BOOST_FOREACH(const Symbol& k, factor->keys()) {
				if (!k.equals(key)) {
					separator.insert(k);
				}
			}
		}

		// add the keys to the rendering
		BOOST_FOREACH(const Symbol& k, separator)
		if (k != key) render += k;

		// combine the factors to get a noisemodel and a combined matrix
		Matrix Ab; SharedDiagonal model;

		boost::tie(Ab, model) = GaussianFactor::combineFactorsAndCreateMatrix(factors,render,dimensions);

	}

	clock_t end = clock ();
	double dif = (double)(end - start) / CLOCKS_PER_SEC;
	return dif;
}


/* ************************************************************************* */
TEST(timeGaussianFactorGraph, linearTime)
{
	// Original T = 1000;

	// Alex's Results
	// T = 100000
	// 1907 (init)    : T - 1.65, 2T = 3.28
	//    int->size_t : T - 1.63, 2T = 3.27
	// 2100           : T - 1.52, 2T = 2.96

	int T = 100000;
	double time1 = timeKalmanSmoother(  T);  cout << "timeKalmanSmoother( T): " << time1;
	double time2 = timeKalmanSmoother(2*T);  cout << "  (2*T): " << time2 << endl;
	DOUBLES_EQUAL(2*time1,time2,0.2);
}


// Timing with N = 30
// 1741: 8.12, 8.12, 8.12, 8.14, 8.16
// 1742: 5.97, 5.97, 5.97, 5.99, 6.02
// 1746: 5.96, 5.96, 5.97, 6.00, 6.04
// 1748: 5.91, 5.92, 5.93, 5.95, 5.96
// 1839: 0.206956 0.206939 0.206213 0.206092 0.206780 // colamd !!!!

// Alex's Machine
// Initial:
// 1907 (N = 30)               :  0.14
//      (N = 100)			   : 16.36
// Improved (int->size_t)
//      (N = 100)              : 15.39
// Using GSL/BLAS for updateAb : 12.87
// Using NoiseQR               : 16.33
// Using correct system        : 10.00

// Switch to 100*100 grid = 10K poses
// 1879: 15.6498 15.3851 15.5279

int size = 100;

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, planar_old)
{
	cout << "Timing planar - original version" << endl;
	double time = timePlanarSmoother(size);
	cout << "timeGaussianFactorGraph : " << time << endl;
	//DOUBLES_EQUAL(5.97,time,0.1);
}

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, planar_new)
{
	cout << "Timing planar - new version" << endl;
	double time = timePlanarSmoother(size, false);
	cout << "timeGaussianFactorGraph : " << time << endl;
	//DOUBLES_EQUAL(5.97,time,0.1);
}

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, planar_eliminate_old)
{
	cout << "Timing planar Eliminate - original version" << endl;
	double time = timePlanarSmootherEliminate(size);
	cout << "timeGaussianFactorGraph : " << time << endl;
	//DOUBLES_EQUAL(5.97,time,0.1);
}

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, planar_eliminate_new)
{
	cout << "Timing planar Eliminate - new version" << endl;
	double time = timePlanarSmootherEliminate(size, false);
	cout << "timeGaussianFactorGraph : " << time << endl;
	//DOUBLES_EQUAL(5.97,time,0.1);
}

size_t reps = 1000;
/* ************************************************************************* */
TEST(timeGaussianFactorGraph, planar_join_old)
{
	cout << "Timing planar join - old" << endl;
	double time = timePlanarSmootherJoinAug(size, reps);
	cout << "timePlanarSmootherJoinAug " << size << " : " << time << endl;
	//DOUBLES_EQUAL(5.97,time,0.1);
}

/* ************************************************************************* */
TEST(timeGaussianFactorGraph, planar_join_new)
{
	cout << "Timing planar join - new" << endl;
	double time = timePlanarSmootherCombined(size, reps);
	cout << "timePlanarSmootherCombined " << size << " : " << time << endl;
	//DOUBLES_EQUAL(5.97,time,0.1);
}


/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */
