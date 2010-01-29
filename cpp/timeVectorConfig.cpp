/*
 * @file timeVectorConfig.cpp
 * @brief Performs timing and profiling for VectorConfig operations
 * @author Frank Dellaert
 */

#include <iostream>
#include <boost/timer.hpp>
#include "VectorConfig.h"

using namespace std;
using namespace gtsam;

/*
 * Results:
 * Frank's machine:
 */
double timeAssignment(size_t n, size_t m, size_t r) {
	// assign a large VectorConfig
	// n =  number of times to loop
	// m =  number of vectors
	// r =  rows per vector

	// Create 2 VectorConfigs
	VectorConfig a, b;
	for (int i = 0; i < m; ++i) {
		Vector v = zero(r);
		Symbol key('x', i);
		a.add(key, v);
		b.add(key, v);
	}

	// start timing
	double elapsed;
	{
		boost::timer t;

		for (int j = 0; j < n; ++j)
			a = b;

		elapsed = t.elapsed();
	}

	return elapsed;
}

int main(int argc, char ** argv) {

	// Time assignment operator
	cout << "Starting operator=() Timing" << endl;
	size_t n = 1000, m = 10000, r = 3;
	double time = timeAssignment(n, m, r);
	cout << "Average Elapsed time for assigning " << m << "*" << r
			<< " VectorConfigs: " << 10e3 * time / n << "ms." << endl;

	return 0;
}
