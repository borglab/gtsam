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

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/IterativeOptimizationParameters.h>

namespace gtsam {

class IterativeSolver {

public:

	typedef IterativeOptimizationParameters Parameters;
	typedef Parameters::shared_ptr sharedParameters;

protected:

	GaussianFactorGraph::shared_ptr graph_;
	VariableIndex::shared_ptr variableIndex_;
	Parameters::shared_ptr parameters_ ;

public:

  IterativeSolver(
    const GaussianFactorGraph::shared_ptr& factorGraph,
    const VariableIndex::shared_ptr& variableIndex):
    graph_(factorGraph), variableIndex_(variableIndex),
    parameters_(new Parameters()) { }

  IterativeSolver(
    const GaussianFactorGraph::shared_ptr& factorGraph,
    const VariableIndex::shared_ptr& variableIndex,
    const Parameters::shared_ptr& parameters):
    graph_(factorGraph), variableIndex_(variableIndex), parameters_(parameters) { }

	IterativeSolver():
		parameters_(new IterativeOptimizationParameters()) {}

	IterativeSolver(const IterativeSolver &solver):
		parameters_(solver.parameters_) {}

	IterativeSolver(const IterativeOptimizationParameters &parameters):
		parameters_(new IterativeOptimizationParameters(parameters)) {}

	IterativeSolver(const sharedParameters parameters):
		parameters_(parameters) {}

	virtual ~IterativeSolver() {}

	virtual VectorValues::shared_ptr optimize () = 0;
};

}
