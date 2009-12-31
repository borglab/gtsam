/*
 * iterative-inl.h
 * @brief Iterative methods, template implementation
 * @author Frank Dellaert
 * Created on: Dec 28, 2009
 */

#include "GaussianFactorGraph.h"
#include "iterative.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class S, class V, class E>
	V conjugateGradients(const S& Ab, V x, bool verbose, double epsilon,
			size_t maxIterations, bool steepest = false) {

		if (maxIterations == 0) maxIterations = dim(x) * (steepest ? 10 : 1);
		size_t reset = (size_t)(sqrt(dim(x))+0.5); // when to reset

		// Start with g0 = A'*(A*x0-b), d0 = - g0
		// i.e., first step is in direction of negative gradient
		V g = Ab.gradient(x);
		V d = -g;
		double dotg0 = dot(g, g), prev_dotg = dotg0;
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
			x = x + alpha * d;
			if (k==maxIterations) break;

			// update gradient (or re-calculate at reset time)
			g = (k%reset==0) ? Ab.gradient(x) : g + alpha * (Ab ^ Ad);

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

} // namespace gtsam
