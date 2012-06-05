/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/IterativeOptimizationParameters.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

class IterativeSolver {

public:

	typedef IterativeOptimizationParameters Parameters;

protected:

	Parameters parameters_ ;

public:

  IterativeSolver(): parameters_() {}
	IterativeSolver(const IterativeSolver &solver) : parameters_(solver.parameters_) {}
	IterativeSolver(const Parameters &parameters) :	parameters_(parameters) {}

	virtual ~IterativeSolver() {}

	virtual VectorValues::shared_ptr optimize () = 0;

	inline const Parameters& parameters() const { return parameters_ ; }
};

}
