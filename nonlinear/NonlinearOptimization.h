/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * NonlinearOptimization.h
 *
 *   Created on: Oct 14, 2010
 *       Author: Kai Ni
 *  Description: Easy interfaces for NonlinearOptimizer
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizationParameters.h>

namespace gtsam {

	/**
	 * all the nonlinear optimization methods
	 */
	enum NonlinearOptimizationMethod {
		LM,            // Levenberg Marquardt
		GN             // Gaussian-Newton
	};

	/**
	 * all the linear solver types
	 */
	enum LinearSolver{
		ELIMINATION,      // Elimination
		MULTIFRONTAL,      // Multi-frontal
		SPCG,			  // Subgraph Preconditioned Conjugate Gradient
	};


	/**
	 * optimization that returns the values
	 */
	template<class G, class T>
	T optimize(const G& graph, const T& initialEstimate, const NonlinearOptimizationParameters& parameters = NonlinearOptimizationParameters(),
			const enum LinearSolver& solver = ELIMINATION);

}

