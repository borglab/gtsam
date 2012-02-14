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

#include <boost/shared_ptr.hpp>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/IterativeOptimizationParameters.h>

namespace gtsam {

class IterativeSolver {

public:

	typedef boost::shared_ptr<IterativeSolver> shared_ptr;
	typedef IterativeOptimizationParameters Parameters;

protected:

	Parameters::shared_ptr parameters_ ;

public:

  IterativeSolver(): parameters_(new Parameters()) {}

	IterativeSolver(const IterativeSolver &solver)
	: parameters_(solver.parameters_) {}

  IterativeSolver(const Parameters::shared_ptr& parameters)
  : parameters_(parameters) {}

	IterativeSolver(const Parameters &parameters)
	:	parameters_(new Parameters(parameters)) {}

	virtual ~IterativeSolver() {}

	virtual VectorValues::shared_ptr optimize () = 0;

	Parameters::shared_ptr parameters() { return parameters_ ; }
};

}
