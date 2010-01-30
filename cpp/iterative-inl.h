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
		double dotg0 = dot(g, g), prev_dotg = dotg0;
		if (dotg0 < epsilon_abs) return x;
		double threshold = epsilon * epsilon * dotg0;

		if (verbose) cout << "CG: epsilon = " << epsilon << ", maxIterations = "
				<< maxIterations << ", ||g0||^2 = " << dotg0 << ", threshold = "
				<< threshold << endl;

		// loop maxIterations times
		for (size_t k = 1;; k++) {

			// calculate optimal step-size
			E Ad = Ab * d;
			double alpha = -dot(d, g) / dot(Ad, Ad);

			// do step in new search direction
			x += alpha * d;
			if (k==maxIterations) break;

			// update gradient (or re-calculate at reset time)
			if (k%reset==0)
				g = Ab.gradient(x);
			else
				axpy(alpha, Ab ^ Ad, g);

			// check for convergence
			double dotg = dot(g, g);
			if (verbose) cout << "iteration " << k << ": alpha = " << alpha
					<< ", dotg = " << dotg << endl;
			if (dotg < threshold) break;

			// calculate new search direction
			if (steepest)
				d = g;
			else {
				double beta = dotg / prev_dotg;
				prev_dotg = dotg;
				d = g + d*beta;
			}
		}
		return x;
	}

/* ************************************************************************* */

} // namespace gtsam
