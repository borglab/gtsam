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

	/**
	 * gradient of objective function 0.5*|Ax-b|^2 at x = A'*(Ax-b)
	 */
	Vector gradient(const System& Ab, const Vector& x) {
		const Matrix& A = Ab.first;
		const Vector& b = Ab.second;
		return A ^ (A * x - b);
	}

	/**
	 * Apply operator A
	 */
	Vector operator*(const System& Ab, const Vector& x) {
		const Matrix& A = Ab.first;
		return A * x;
	}

	/**
	 * Apply operator A^T
	 */
	Vector operator^(const System& Ab, const Vector& x) {
		const Matrix& A = Ab.first;
		return A ^ x;
	}

	/* ************************************************************************* */
	// Method of conjugate gradients (CG) template
	// "System" class S needs gradient(S,v), e=S*v, v=S^e
	// "Vector" class V needs dot(v,v), -v, v+v, s*v
	// "Vector" class E needs dot(v,v)
	template<class S, class V, class E>
	V CGD(const S& Ab, V x, double threshold = 1e-9) {

		// Start with g0 = A'*(A*x0-b), d0 = - g0
		// i.e., first step is in direction of negative gradient
		V g = gradient(Ab, x);
		V d = -g;
		double prev_dotg = dot(g, g);

		// loop max n times
		size_t n = x.size();
		for (int k = 1; k <= n; k++) {

			// calculate optimal step-size
			E Ad = Ab * d;
			double alpha = -dot(d, g) / dot(Ad, Ad);

			// do step in new search direction
			x = x + alpha * d;
			if (k == n) break;

			// update gradient
			g = g + alpha * (Ab ^ Ad);

			// check for convergence
			double dotg = dot(g, g);
			if (dotg < threshold) break;

			// calculate new search direction
			double beta = dotg / prev_dotg;
			prev_dotg = dotg;
			d = -g + beta * d;
		}
		return x;
	}

	/* ************************************************************************* */
	Vector conjugateGradientDescent(const System& Ab, const Vector& x,
			double threshold) {
		return CGD<System, Vector, Vector> (Ab, x);
	}

	/* ************************************************************************* */
	Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
			const Vector& x, double threshold) {
		System Ab = make_pair(A, b);
		return CGD<System, Vector, Vector> (Ab, x);
	}

	/* ************************************************************************* */
	VectorConfig gradient(const GaussianFactorGraph& fg, const VectorConfig& x) {
		return fg.gradient(x);
	}

	/* ************************************************************************* */
	VectorConfig conjugateGradientDescent(const GaussianFactorGraph& fg,
			const VectorConfig& x, double threshold) {
		return CGD<GaussianFactorGraph, VectorConfig, Errors> (fg, x);
	}

/* ************************************************************************* */

} // namespace gtsam
