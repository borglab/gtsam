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

#define TIME(STATEMENT) { boost::timer t; \
		for (int j = 0; j < n; ++j) STATEMENT; \
		double time = t.elapsed(); \
		cout << "Average elapsed time :" << 10e3 * time / n << "ms." << endl; }

/* ************************************************************************* */
void unsafe_assign(VectorConfig& a, const VectorConfig& b) {
	VectorConfig::const_iterator bp = b.begin();
	for (VectorConfig::iterator ap = a.begin(); ap != a.end(); ap++, bp++)
		ap->second = bp->second;
}

/* ************************************************************************* */
void unsafe_add(VectorConfig& a, const VectorConfig& b) {
	VectorConfig::const_iterator bp = b.begin();
	for (VectorConfig::iterator ap = a.begin(); ap != a.end(); ap++, bp++)
		ap->second += bp->second;
}

/* ************************************************************************* */
int main(int argc, char ** argv) {

	// Time assignment operator
	cout << "Starting operator=() Timing" << endl;

	// n =  number of times to loop
	// m =  number of vectors
	// r =  rows per vector
	size_t n = 100, m = 10000, r = 3, alpha = 0.1;

	// Create 2 VectorConfigs
	VectorConfig x, y;
	for (int i = 0; i < m; ++i) {
		Vector v = zero(r);
		Symbol key('x', i);
		x.add(key, v);
		y.add(key, v);
	}

	cout << "Convenient VectorConfig:" << endl;
	TIME(x=y);
	TIME(x+=y);
	TIME(x+=alpha*y);
	//TIME(a=a+b);

	cout << "Unsafe VectorConfig:" << endl;
	TIME(unsafe_assign(x,y));
	TIME(unsafe_add(x,y));
	TIME(axpy(alpha,x,y));

	cout << "Compares with Vector:" << endl;
	Vector v = zero(m * r), w = zero(m * r);
	TIME(v=w);
	TIME(v+=w);
	TIME(v+=alpha*w);
	TIME(axpy(alpha,v,w));
	//	TIME(v=v+w);

	return 0;
}
/* ************************************************************************* */
