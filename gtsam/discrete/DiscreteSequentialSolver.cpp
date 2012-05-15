/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteSequentialSolver.cpp
 * @date Feb 16, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

//#define ENABLE_TIMING
#include <gtsam/discrete/DiscreteSequentialSolver.h>
#include <gtsam/inference/GenericSequentialSolver-inl.h>
#include <gtsam/base/timing.h>

namespace gtsam {

	template class GenericSequentialSolver<DiscreteFactor> ;

	/* ************************************************************************* */
	DiscreteFactor::sharedValues DiscreteSequentialSolver::optimize() const {

		static const bool debug = false;

		if (debug) this->factors_->print("DiscreteSequentialSolver, eliminating ");
		if (debug) this->eliminationTree_->print(
				"DiscreteSequentialSolver, elimination tree ");

		// Eliminate using the elimination tree
		tic(1, "eliminate");
		DiscreteBayesNet::shared_ptr bayesNet = eliminate();
		toc(1, "eliminate");

		if (debug) bayesNet->print("DiscreteSequentialSolver, Bayes net ");

		// Allocate the solution vector if it is not already allocated

		// Back-substitute
		tic(2, "optimize");
		DiscreteFactor::sharedValues solution = gtsam::optimize(*bayesNet);
		toc(2, "optimize");

		if (debug) solution->print("DiscreteSequentialSolver, solution ");

		return solution;
	}
/* ************************************************************************* */

}
