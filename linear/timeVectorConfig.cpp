/*
 * @file timeVectorConfig.cpp
 * @brief Performs timing and profiling for VectorConfig operations
 * @author Frank Dellaert
 */

#include <iostream>
#include <boost/timer.hpp>
#include "VectorBTree.h"
#include "VectorMap.h"

using namespace std;
using namespace gtsam;

#define TIME1(STATEMENT) { boost::timer t; \
		STATEMENT; \
		double time = t.elapsed(); \
		cout << "Average elapsed time :" << 10e3 * time / n << "ms." << endl; }

#define TIME(STATEMENT) TIME1(for (size_t j = 0; j < n; ++j) STATEMENT;)

/* ************************************************************************* */
int main(int argc, char ** argv) {

	// Time assignment operator
	cout << "Starting operator=() Timing" << endl;

	// n =  number of times to loop
	// m =  number of vectors
	// r =  rows per vector
	size_t n = 100, m = 30000, r = 3, alpha = 0.1;

	{
		cout << "Vector:" << endl;
		Vector v = zero(m * r), w = zero(m * r);
		TIME(v=w);
		TIME(v+=w);
		TIME(v+=alpha*w);
		TIME(axpy(alpha,v,w));
	}

	{
		// Create 2 VectorBTrees and one VectorMap
		VectorBTree p, q;
		VectorMap old;
		cout << "Creating VectorBTree:" << endl;
		TIME1(for (size_t i = 0; i < m; ++i) {
			Vector v = zero(r);
			Symbol key('x', i);
			p.insert(key, v);
			q.insert(key, v);
			old.insert(key, v);
		})

		cout << "VectorBTree:" << endl;
		TIME(p=q);
		TIME(p+=q);
		TIME(p+=alpha*q);
		TIME(axpy(alpha,p,q));

		cout << "VectorBTree get:" << endl;
		TIME1(for (size_t i = 0; i < m; ++i) p[Symbol('x', i)]);

		cout << "VectorMap get:" << endl;
		TIME1(for (size_t i = 0; i < m; ++i) old[Symbol('x', i)]);
}

	return 0;
}
/* ************************************************************************* */
