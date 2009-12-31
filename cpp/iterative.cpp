/*
 * iterative.cpp
 * @brief Iterative methods, implementation
 * @author Frank Dellaert
 * Created on: Dec 28, 2009
 */

#include "GaussianFactorGraph.h"
#include "iterative-inl.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	Vector steepestDescent(const System& Ab, const Vector& x, bool verbose,
			double epsilon, size_t maxIterations) {
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				maxIterations, true);
	}

	Vector conjugateGradientDescent(const System& Ab, const Vector& x,
			bool verbose, double epsilon, size_t maxIterations) {
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				maxIterations);
	}

	/* ************************************************************************* */
	Vector steepestDescent(const Matrix& A, const Vector& b, const Vector& x,
			bool verbose, double epsilon, size_t maxIterations) {
		System Ab(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				maxIterations, true);
	}

	Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
			const Vector& x, bool verbose, double epsilon, size_t maxIterations) {
		System Ab(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				maxIterations);
	}

	/* ************************************************************************* */
	VectorConfig steepestDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, bool verbose, double epsilon, size_t maxIterations) {
		return conjugateGradients<GaussianFactorGraph, VectorConfig, Errors> (fg,
				x, verbose, epsilon, maxIterations, true);
	}

	VectorConfig conjugateGradientDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, bool verbose, double epsilon, size_t maxIterations) {
		return conjugateGradients<GaussianFactorGraph, VectorConfig, Errors> (fg,
				x, verbose, epsilon, maxIterations);
	}

	/* ************************************************************************* */

} // namespace gtsam
