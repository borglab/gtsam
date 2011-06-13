/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IterativeSolver.h
 * @date Oct 24, 2010
 * @author Yong-Dian Jian
 * @brief Base Class for all iterative solvers of linear systems
 */

#pragma once

#include <gtsam/linear/IterativeOptimizationParameters.h>

namespace gtsam {

class IterativeSolver {

public:
	typedef IterativeOptimizationParameters Parameters;
	typedef boost::shared_ptr<Parameters> sharedParameters;

	sharedParameters parameters_ ;

	IterativeSolver():
		parameters_(new IterativeOptimizationParameters()) {}

	IterativeSolver(const IterativeSolver &solver):
		parameters_(solver.parameters_) {}

	IterativeSolver(const IterativeOptimizationParameters &parameters):
		parameters_(new IterativeOptimizationParameters(parameters)) {}

	IterativeSolver(const sharedParameters parameters):
		parameters_(parameters) {}
};

}
