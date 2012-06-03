/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/IterativeOptimizationParameters.h>
#include <boost/shared_ptr.hpp>

namespace gtsam {

class IterativeSolver {

public:

	typedef boost::shared_ptr<IterativeSolver> shared_ptr;
	typedef IterativeOptimizationParameters Parameters;

protected:

	Parameters::shared_ptr parameters_ ;

public:

  IterativeSolver(): parameters_(new Parameters()) {}
	IterativeSolver(const IterativeSolver &solver) : parameters_(solver.parameters_) {}
  IterativeSolver(const Parameters::shared_ptr& parameters) : parameters_(parameters) {}
	IterativeSolver(const Parameters &parameters) :	parameters_(new Parameters(parameters)) {}

	virtual ~IterativeSolver() {}

	virtual VectorValues::shared_ptr optimize () = 0;

	inline Parameters::shared_ptr parameters() { return parameters_ ; }
};

}
