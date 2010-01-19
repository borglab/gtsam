/*
 * @file timeMatrix.cpp
 * @brief Performs timing and profiling for Matrix operations
 * @author Alex Cunningham
 */

#include <iostream>
#include <boost/timer.hpp>
#include "Matrix.h"

using namespace std;
using namespace gtsam;

/*
 * Results:
 * Alex's Machine:
 *  - no pass: 0.1818 sec
 *  - pass   : 0.1802 sec
 */
double timeCollect(size_t p, size_t m, size_t n, bool passDims, size_t reps) {
	// create a large number of matrices
	// p =  number of matrices
	// m =  rows per matrix
	// n =  columns per matrix
	// reps = number of repetitions

	// fill the matrices with identities
	vector<const Matrix *> matrices;
	for (int i=0; i<p;++i) {
		Matrix * M = new Matrix;
		(*M) = eye(m,n);
		matrices.push_back(M);
	}

	// start timing
	Matrix result;
	double elapsed;
	{
		boost::timer t;

		if (passDims)
			for (int i=0; i<reps; ++i)
				result = collect(matrices, m, n);
		else
			for (int i=0; i<reps; ++i)
				result = collect(matrices);
		elapsed = t.elapsed();
	}
	// delete the matrices
	for (int i=0; i<p;++i) {
		delete matrices[i];
	}

	return elapsed/reps;
}

int main(int argc, char ** argv) {

	// Time collect()
	cout << "Starting Matrix::collect() Timing" << endl;
	size_t p = 100000; size_t m = 10; size_t n = 12; size_t reps = 50;
	double collect_time1 = timeCollect(p, m, n, false, reps);
	double collect_time2 = timeCollect(p, m, n, true, reps);
	cout << "Elapsed time for collect (no pass) [" << p << " (" << m << ", " << n << ") matrices] : " << collect_time1 << endl;
	cout << "Elapsed time for collect (pass)    [" << p << " (" << m << ", " << n << ") matrices] : " << collect_time2 << endl;



	return 0;
}
