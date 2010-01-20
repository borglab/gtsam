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
 *  - (1st pass of simple changes) no pass: 0.184 sec , pass: 0.181 sec
 *  - (1st rev memcpy)             no pass: 0.181 sec , pass: 0.180 sec
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

/*
 * Results:
 * Alex's Machine:
 *  - Original: 0.60 sec (x1000)
 */
double timeVScaleColumn(size_t m, size_t n, size_t reps) {
	// make a matrix to scale
	Matrix M(m, n);
	for (int i=0; i<m; ++i)
		for (int j=0; j<n; ++j)
			M(i,j) = 2*i+j;

	// make a vector to use for scaling
	Vector V(m);
	for (int i=0; i<m; ++i)
		V(i) = i*2;

	double elapsed;
	Matrix result;
	{
		boost::timer t;
		for (int i=0; i<reps; ++i)
			Matrix result = vector_scale(M,V);
		elapsed = t.elapsed();
	}

	return elapsed;
}

/*
 * Results:
 * Alex's Machine:
 *  - Original: 0.54 sec (x1000)
 */
double timeVScaleRow(size_t m, size_t n, size_t reps) {
	// make a matrix to scale
	Matrix M(m, n);
	for (int i=0; i<m; ++i)
		for (int j=0; j<n; ++j)
			M(i,j) = 2*i+j;

	// make a vector to use for scaling
	Vector V(n);
	for (int i=0; i<n; ++i)
		V(i) = i*2;

	double elapsed;
	Matrix result;
	{
		boost::timer t;
		for (int i=0; i<reps; ++i)
			Matrix result = vector_scale(V,M);
		elapsed = t.elapsed();
	}

	return elapsed;
}

int main(int argc, char ** argv) {

	// Time collect()
	cout << "Starting Matrix::collect() Timing" << endl;
	size_t p = 100000; size_t m = 10; size_t n = 12; size_t reps = 50;
	double collect_time1 = timeCollect(p, m, n, false, reps);
	double collect_time2 = timeCollect(p, m, n, true, reps);
	cout << "Average Elapsed time for collect (no pass) [" << p << " (" << m << ", " << n << ") matrices] : " << collect_time1 << endl;
	cout << "Average Elapsed time for collect (pass)    [" << p << " (" << m << ", " << n << ") matrices] : " << collect_time2 << endl;

	// Time vector_scale_column
	cout << "Starting Matrix::vector_scale(column) Timing" << endl;
	size_t m1 = 400; size_t n1 = 480; size_t reps1 = 1000;
	double vsColumn_time = timeVScaleColumn(m1, n1, reps1);
	cout << "Elapsed time for vector_scale(column) [(" << m1 << ", " << n1 << ") matrix] : " << vsColumn_time << endl;

	// Time vector_scale_row
	cout << "Starting Matrix::vector_scale(row)    Timing" << endl;
	double vsRow_time = timeVScaleRow(m1, n1, reps1);
	cout << "Elapsed time for vector_scale(row)    [(" << m1 << ", " << n1 << ") matrix] : " << vsRow_time << endl;

	return 0;
}
