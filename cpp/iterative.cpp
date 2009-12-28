/*
 * iterative.cpp
 * @brief Iterative methods, implementation
 * @author Frank Dellaert
 * Created on: Dec 28, 2009
 */

#include "GaussianFactorGraph.h"
#include "iterative.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	// Method of conjugate gradients (CG) template
	// "System" class S needs gradient(S,v), e=S*v, v=S^e
	// "Vector" class V needs dot(v,v), -v, v+v, s*v
	// "Vector" class E needs dot(v,v)
	// if (steepest) does steepest descent
	template<class S, class V, class E>
	V conjugateGradients(const S& Ab, V x, bool verbose, double epsilon,
			size_t maxIterations, bool steepest = false) {

		if (maxIterations == 0) maxIterations = dim(x) * (steepest ? 10 : 1);

		// Start with g0 = A'*(A*x0-b), d0 = - g0
		// i.e., first step is in direction of negative gradient
		V g = gradient(Ab, x);
		V d = -g;
		double dotg0 = dot(g, g), prev_dotg = dotg0;
		double threshold = epsilon * epsilon * dotg0;

		if (verbose) cout << "CG: epsilon = " << epsilon << ", maxIterations = "
				<< maxIterations << ", ||g0||^2 = " << dotg0 << ", threshold = "
				<< threshold << endl;

		// loop maxIterations times
		for (size_t k = 0; k < maxIterations; k++) {

			// calculate optimal step-size
			E Ad = Ab * d;
			double alpha = -dot(d, g) / dot(Ad, Ad);

			// do step in new search direction
			x = x + alpha * d;

			// update gradient
			g = g + alpha * (Ab ^ Ad);

			// check for convergence
			double dotg = dot(g, g);
			if (verbose) cout << "iteration " << k << ": alpha = " << alpha
					<< ", dotg = " << dotg << endl;
			if (dotg < threshold) break;

			// calculate new search direction
			if (steepest)
				d = -g;
			else {
				double beta = dotg / prev_dotg;
				prev_dotg = dotg;
				d = -g + beta * d;
			}
		}
		return x;
	}

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
