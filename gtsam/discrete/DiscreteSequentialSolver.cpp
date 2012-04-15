/*
 * DiscreteSequentialSolver.cpp
 *
 *  @date Feb 16, 2011
 *  @author Duy-Nguyen Ta
 */

//#define ENABLE_TIMING
#include <gtsam2/discrete/DiscreteSequentialSolver.h>
#include <gtsam2/discrete/PotentialTable.h>
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
