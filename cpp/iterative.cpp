/*
 * iterative.cpp
 * @brief Iterative methods, implementation
 * @author Frank Dellaert
 * Created on: Dec 28, 2009
 */
#include <iostream>

#include "Vector.h"
#include "Matrix.h"
#include "GaussianFactorGraph.h"
#include "iterative-inl.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	void System::print (const string& s) const {
		cout << s << endl;
		gtsam::print(A_, "A");
		gtsam::print(b_, "b");
	}

	/* ************************************************************************* */
	Vector steepestDescent(const System& Ab, const Vector& x, bool verbose,
			double epsilon, double epsilon_abs, size_t maxIterations) {
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				epsilon_abs, maxIterations, true);
	}

	Vector conjugateGradientDescent(const System& Ab, const Vector& x,
			bool verbose, double epsilon, double epsilon_abs, size_t maxIterations) {
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				epsilon_abs, maxIterations);
	}

	/* ************************************************************************* */
	Vector steepestDescent(const Matrix& A, const Vector& b, const Vector& x,
			bool verbose, double epsilon, double epsilon_abs, size_t maxIterations) {
		System Ab(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				epsilon_abs, maxIterations, true);
	}

	Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
			const Vector& x, bool verbose, double epsilon, double epsilon_abs, size_t maxIterations) {
		System Ab(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				epsilon_abs, maxIterations);
	}

	/* ************************************************************************* */
	VectorConfig steepestDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, bool verbose, double epsilon, double epsilon_abs, size_t maxIterations) {
		return conjugateGradients<GaussianFactorGraph, VectorConfig, Errors> (fg,
				x, verbose, epsilon, epsilon_abs, maxIterations, true);
	}

	VectorConfig conjugateGradientDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, bool verbose, double epsilon, double epsilon_abs, size_t maxIterations) {
		return conjugateGradients<GaussianFactorGraph, VectorConfig, Errors> (fg,
				x, verbose, epsilon, epsilon_abs, maxIterations);
	}

	/* ************************************************************************* */

} // namespace gtsam
