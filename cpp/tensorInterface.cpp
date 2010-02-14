/*
 * tensorInterface.cpp
 * @brief Interfacing tensors template library and gtsam
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#include "tensorInterface.h"

using namespace std;
using namespace tensors;

namespace gtsam {

	boost::tuple<int, double, Vector> DLT(const Matrix& A) {

		// Do SVD on A
		Matrix U, V;
		Vector S;
		svd(A, U, S, V);

		// Find minimum column
		int n = V.size2(), min_j = n - 1, rank = 0;
		for (int j = 0; j < n; j++)
			if (S(j) > 1e-9) rank++;
		double min_S = S(min_j);
		for (int j = 0; j < n - 1; j++)
			if (S(j) < min_S) {
				min_j = j;
				min_S = S(j);
			}

		// Return minimum column
		return boost::tuple<int, double, Vector>(rank, min_S, column_(V, min_j));
	}

} // namespace gtsam
