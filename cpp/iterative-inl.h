/*
 * iterative-inl.h
 * @brief Iterative methods, template implementation
 * @author Frank Dellaert
 * Created on: Dec 28, 2009
 */

#pragma once

#include "GaussianFactorGraph.h"
#include "iterative.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	/**
	 * conjugate gradient method.
	 * S: linear system, V: step vector, E: errors
	 */
	template<class S, class V, class E>
	V conjugateGradients(const S& Ab, V x, bool verbose, double epsilon, double epsilon_abs,
			size_t maxIterations, bool steepest = false) {
		if (maxIterations == 0) maxIterations = dim(x) * (steepest ? 10 : 1);
		size_t reset = (size_t)(sqrt(dim(x))+0.5); // when to reset

		// Start with g0 = A'*(A*x0-b), d0 = - g0
		// i.e., first step is in direction of negative gradient
		V g = Ab.gradient(x);
		V d = g; // instead of negating gradient, alpha will be negated
		double gamma0 = dot(g, g), gamma_old = gamma0;
		if (gamma0 < epsilon_abs) return x;
		double threshold = epsilon * epsilon * gamma0;

		if (verbose) cout << "CG: epsilon = " << epsilon << ", maxIterations = "
				<< maxIterations << ", ||g0||^2 = " << gamma0 << ", threshold = "
				<< threshold << endl;

		// Allocate and calculate A*d for first iteration
		E Ad = Ab * d;

		// loop maxIterations times
		for (size_t k = 1;; k++) {

			// calculate optimal step-size
			double alpha = - dot(d, g) / dot(Ad, Ad);

			// do step in new search direction
			axpy(alpha, d, x); // x += alpha*d
			if (k==maxIterations) break;

			// update gradient (or re-calculate at reset time)
			if (k%reset==0)
				g = Ab.gradient(x);
			else
				// axpy(alpha, Ab ^ Ad, g);  // g += alpha*(Ab^Ad)
				Ab.transposeMultiplyAdd(alpha, Ad, g);

			// check for convergence
			double gamma = dot(g, g);
			if (verbose) cout << "iteration " << k << ": alpha = " << alpha
					<< ", dotg = " << gamma << endl;
			if (gamma < threshold) break;

			// calculate new search direction
			if (steepest)
				d = g;
			else {
				double beta = gamma / gamma_old;
				gamma_old = gamma;
				// d = g + d*beta;
				scal(beta,d);
				axpy(1.0, g, d);
			}

			// In-place recalculation Ad <- A*d to avoid re-allocating Ad
			Ab.multiplyInPlace(d,Ad);
		}
		return x;
	}

/* ************************************************************************* */

} // namespace gtsam
