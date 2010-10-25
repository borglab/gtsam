/*
 * IterativeSolver.h
 *
 *  Created on: Oct 24, 2010
 *      Author: Yong-Dian Jian
 *
 *  Base Class for all iterative solvers of linear systems
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

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

	IterativeSolver(const sharedParameters parameters):
		parameters_(parameters) {}
};

}
