/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimizer.cpp
 * @brief Convergence functions not dependent on graph types
 * @author Frank Dellaert
 * @date Jul 17, 2010
 */

#include <iostream>
#include <iomanip>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
NonlinearOptimizer::shared_ptr NonlinearOptimizer::update(const NonlinearOptimizer::SharedGraph& newGraph) const {
  shared_ptr result(this->clone());
  result->graph_ = newGraph;
  return result;
}

/* ************************************************************************* */
NonlinearOptimizer::shared_ptr NonlinearOptimizer::update(const NonlinearOptimizer::SharedParams& newParams) const {
  shared_ptr result(this->clone());
  result->params_ = newParams;
  return result;
}

/* ************************************************************************* */
NonlinearOptimizer::SharedState NonlinearOptimizer::defaultOptimize(const SharedState& initial) const {

  const SharedParams& params = this->params();
  double currentError = initial->error();

  // check if we're already close enough
  if(currentError <= params->errorTol) {
    if (params->verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params->errorTol << endl;
    return initial;
  }

  // Maybe show output
  if (params->verbosity >= NonlinearOptimizerParams::VALUES) this->values()->print("Initial values");
  if (params->verbosity >= NonlinearOptimizerParams::ERROR) cout << "Initial error: " << this->error() << endl;

  // Return if we already have too many iterations
  if(this->iterations() >= params->maxIterations)
    return initial;

  // Iterative loop
  SharedState next = initial;
  do {
    // Do next iteration
    currentError = next->error;
    next = this->iterate(next);

    // Maybe show output
    if (params->verbosity >= NonlinearOptimizerParams::VALUES) next->values->print("newValues");
    if (params->verbosity >= NonlinearOptimizerParams::ERROR) cout << "newError: " << next->error << endl;
  } while(next->iterations < params->maxIterations &&
      !checkConvergence(params->relativeErrorTol, params->absoluteErrorTol,
            params->errorTol, currentError, next->error, params->verbosity));

  // Printing if verbose
  if (params->verbosity >= NonlinearOptimizerParams::ERROR &&
      next->iterations >= params->maxIterations)
    cout << "Terminating because reached maximum iterations" << endl;

  // Return optimizer from final iteration
  return next;
}

/* ************************************************************************* */
void NonlinearOptimizer::defaultInitialState(NonlinearOptimizerState& initial) const {
  state.values = initialValues;
  state.error = graph_->error(initialValues);
  state.iterations = 0;
}

/* ************************************************************************* */
bool checkConvergence(double relativeErrorTreshold, double absoluteErrorTreshold,
    double errorThreshold, double currentError, double newError,
    NonlinearOptimizerParams::Verbosity verbosity) {

	if ( verbosity >= 2 ) {
		if ( newError <= errorThreshold )
			cout << "errorThreshold: " << newError << " < " << errorThreshold << endl;
		else
			cout << "errorThreshold: " << newError << " > " << errorThreshold << endl;
	}

	if ( newError <= errorThreshold ) return true ;

	// check if diverges
	double absoluteDecrease = currentError - newError;
	if (verbosity >= 2) {
		if (absoluteDecrease <= absoluteErrorTreshold)
			cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " < " << absoluteErrorTreshold << endl;
		else
			cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " >= " << absoluteErrorTreshold << endl;
	}

	// calculate relative error decrease and update currentError
	double relativeDecrease = absoluteDecrease / currentError;
	if (verbosity >= 2) {
		if (relativeDecrease <= relativeErrorTreshold)
			cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " < " << relativeErrorTreshold << endl;
		else
			cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " >= " << relativeErrorTreshold << endl;
	}
	bool converged = (relativeErrorTreshold && (relativeDecrease <= relativeErrorTreshold))
			|| (absoluteDecrease <= absoluteErrorTreshold);
	if (verbosity >= 1 && converged) {
		if(absoluteDecrease >= 0.0)
		  cout << "converged" << endl;
		else
		  cout << "Warning:  stopping nonlinear iterations because error increased" << endl;
	}
	return converged;
}


}
