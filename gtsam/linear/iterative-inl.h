/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * iterative-inl.h
 * @brief Iterative methods, template implementation
 * @author Frank Dellaert
 * Created on: Dec 28, 2009
 */

#pragma once

#include <boost/shared_ptr.hpp>

#include <gtsam/linear/IterativeOptimizationParameters.h>
#include <gtsam/linear/iterative.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	// state for CG method
	template<class S, class V, class E>
	struct CGState {

		typedef IterativeOptimizationParameters Parameters;
		const Parameters &parameters_;

		int k;
		bool steepest;
		V g, d;
		double gamma, threshold;
		E Ad;

		/* ************************************************************************* */
		// Constructor
		CGState(const S& Ab, const V& x, const Parameters &parameters, bool steep):
		parameters_(parameters),k(0),steepest(steep) {

			// Start with g0 = A'*(A*x0-b), d0 = - g0
			// i.e., first step is in direction of negative gradient
			g = gradient(Ab,x);
			d = g; // instead of negating gradient, alpha will be negated

			// init gamma and calculate threshold
			gamma = dot(g,g) ;
			threshold = ::max(parameters_.epsilon_abs(), parameters_.epsilon() * parameters_.epsilon() * gamma);

			// Allocate and calculate A*d for first iteration
			if (gamma > parameters_.epsilon_abs()) Ad = Ab * d;
		}

		/* ************************************************************************* */
		// print
		void print(const V& x) {
			cout << "iteration = " << k << endl;
			gtsam::print(x,"x");
			gtsam::print(g, "g");
			cout << "dotg = " << gamma << endl;
			gtsam::print(d, "d");
			gtsam::print(Ad, "Ad");
		}

		/* ************************************************************************* */
		// step the solution
		double takeOptimalStep(V& x) {
			// TODO: can we use gamma instead of dot(d,g) ????? Answer not trivial
			double alpha = -dot(d, g) / dot(Ad, Ad); // calculate optimal step-size
			axpy(alpha, d, x); // // do step in new search direction, x += alpha*d
			return alpha;
		}

		/* ************************************************************************* */
		// take a step, return true if converged
		bool step(const S& Ab, V& x) {
			if ((++k) >= parameters_.maxIterations()) return true;

			//---------------------------------->
			double alpha = takeOptimalStep(x);

			// update gradient (or re-calculate at reset time)
			if (k % parameters_.reset() == 0) g = gradient(Ab,x);
			// axpy(alpha, Ab ^ Ad, g);  // g += alpha*(Ab^Ad)
			else transposeMultiplyAdd(Ab, alpha, Ad, g);

			// check for convergence
			double new_gamma = dot(g, g);
			if (parameters_.verbosity() != IterativeOptimizationParameters::SILENT)
				cout << "iteration " << k << ": alpha = " << alpha
				     << ", dotg = " << new_gamma << endl;
			if (new_gamma < threshold) return true;

			// calculate new search direction
			if (steepest) d = g;
			else {
				double beta = new_gamma / gamma;
				// d = g + d*beta;
				scal(beta, d);
				axpy(1.0, g, d);
			}

			gamma = new_gamma;

			// In-place recalculation Ad <- A*d to avoid re-allocating Ad
			multiplyInPlace(Ab, d, Ad);
			return false;
		}

	}; // CGState Class

	/* ************************************************************************* */
	// conjugate gradient method.
	// S: linear system, V: step vector, E: errors
	template<class S, class V, class E>
	V conjugateGradients(
			const S& Ab,
			V x,
			const IterativeOptimizationParameters &parameters,
			bool steepest = false) {

		CGState<S, V, E> state(Ab, x, parameters, steepest);
		if (parameters.verbosity() != IterativeOptimizationParameters::SILENT)
			cout << "CG: epsilon = " << parameters.epsilon()
				 << ", maxIterations = " << parameters.maxIterations()
				 << ", ||g0||^2 = " << state.gamma
				 << ", threshold = " << state.threshold << endl;

		if ( state.gamma < state.threshold ) {
			if (parameters.verbosity() != IterativeOptimizationParameters::SILENT)
				cout << "||g0||^2 < threshold, exiting immediately !" << endl;

			return x;
		}

		// loop maxIterations times
		while (!state.step(Ab, x)) {}
		return x;
	}
/* ************************************************************************* */

} // namespace gtsam
