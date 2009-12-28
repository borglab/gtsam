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

	/** gradient of objective function 0.5*|Ax-b|^2 at x = A'*(Ax-b) */
	Vector gradient(const System& Ab, const Vector& x) {
		const Matrix& A = Ab.first;
		const Vector& b = Ab.second;
		return A ^ (A * x - b);
	}

	/** Apply operator A */
	Vector operator*(const System& Ab, const Vector& x) {
		const Matrix& A = Ab.first;
		return A * x;
	}

	/** Apply operator A^T */
	Vector operator^(const System& Ab, const Vector& x) {
		const Matrix& A = Ab.first;
		return A ^ x;
	}

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
		System Ab = make_pair(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				maxIterations, true);
	}

	Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
			const Vector& x, bool verbose, double epsilon, size_t maxIterations) {
		System Ab = make_pair(A, b);
		return conjugateGradients<System, Vector, Vector> (Ab, x, verbose, epsilon,
				maxIterations);
	}

	/* ************************************************************************* */
	VectorConfig gradient(const GaussianFactorGraph& fg, const VectorConfig& x) {
		return fg.gradient(x);
	}

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
