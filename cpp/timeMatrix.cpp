/*
 * @file timeMatrix.cpp
 * @brief Performs timing and profiling for Matrix operations
 * @author Alex Cunningham
 */

#include <iostream>
#include <boost/progress.hpp>
#include "Matrix.h"

using namespace std;
using namespace gtsam;

int main(int argc, char ** argv) {
	cout << "Starting Matrix::collect() Timing" << endl;

	// create a large number of matrices
	size_t n, m, p;
	p = 100000; // number of matrices
	n = 10;  // rows per matrix
	m = 12;  // columns per matrix

	// fill the matrices with identities
	vector<const Matrix *> matrices;
	for (int i=0; i<p;++i) {
		Matrix * M = new Matrix;
		(*M) = eye(m,n);
		matrices.push_back(M);
	}

	// start timing
	Matrix result;
	{
		boost::progress_timer t;
		result = collect(matrices);
	}

	// delete the matrices
	for (int i=0; i<p;++i) {
		delete matrices[i];
	}

	return 0;
}
