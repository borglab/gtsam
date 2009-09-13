/**
 * @file    NonlinearFactorGraph.cpp
 * @brief   Factor Graph Constsiting of non-linear factors
 * @brief   nonlinearFactorGraph
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <math.h>
#include <climits>
#include <stdexcept>
#include <boost/tuple/tuple.hpp>
#include "NonlinearFactorGraph.h" 

using namespace std;
namespace gtsam {

	/* ************************************************************************* */
	LinearFactorGraph NonlinearFactorGraph::linearize(const FGConfig& config) const {
		// TODO speed up the function either by returning a pointer or by
		// returning the linearisation as a second argument and returning
		// the reference

		// create an empty linear FG
		LinearFactorGraph linearFG;

		// linearize all factors
		for (const_iterator factor = factors_.begin(); factor < factors_.end(); factor++) {
			LinearFactor::shared_ptr lf = (*factor)->linearize(config);
			linearFG.push_back(lf);
		}

		return linearFG;
	}

	/* ************************************************************************* */
	double calculate_error(const NonlinearFactorGraph& fg,
			const FGConfig& config, int verbosity) {
		double newError = fg.error(config);
		if (verbosity >= 1)
			cout << "error: " << newError << endl;
		return newError;
	}

	/* ************************************************************************* */
	bool check_convergence(double relativeErrorTreshold,
			double absoluteErrorTreshold, double currentError, double newError,
			int verbosity) {
		// check if diverges
		double absoluteDecrease = currentError - newError;
		if (verbosity >= 2)
			cout << "absoluteDecrease: " << absoluteDecrease << endl;
		if (absoluteDecrease < 0)
			throw overflow_error(
					"NonlinearFactorGraph::optimize: error increased, diverges.");

		// calculate relative error decrease and update currentError
		double relativeDecrease = absoluteDecrease / currentError;
		if (verbosity >= 2)
			cout << "relativeDecrease: " << relativeDecrease << endl;
		bool converged = (relativeDecrease < relativeErrorTreshold)
				|| (absoluteDecrease < absoluteErrorTreshold);
		if (verbosity >= 1 && converged)
			cout << "converged" << endl;
		return converged;
	}

	/* ************************************************************************* */
	bool NonlinearFactorGraph::check_convergence(const FGConfig& config1,
			const FGConfig& config2, double relativeErrorTreshold,
			double absoluteErrorTreshold, int verbosity) const {
		double currentError = calculate_error(*this, config1, verbosity);
		double newError = calculate_error(*this, config2, verbosity);
		return gtsam::check_convergence(relativeErrorTreshold,
				absoluteErrorTreshold, currentError, newError, verbosity);

	}

/* ************************************************************************* */

} // namespace gtsam
