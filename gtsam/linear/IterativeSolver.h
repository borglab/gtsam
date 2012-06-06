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

  IterativeSolver(){}
	virtual ~IterativeSolver() {}

	virtual VectorValues::shared_ptr optimize () = 0;
	virtual const IterativeOptimizationParameters& _params() const = 0;
};

}
