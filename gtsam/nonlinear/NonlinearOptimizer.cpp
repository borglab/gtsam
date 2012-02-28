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
NonlinearOptimizer::auto_ptr NonlinearOptimizer::defaultOptimize() const {

  double currentError = this->error();

  // check if we're already close enough
  if(currentError <= params_->errorTol) {
    if (params_->verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params_->errorTol << endl;
    return this->clone();
  }

  // Return if we already have too many iterations
  if(this->iterations() >= params_->maxIterations)
    return this->clone();

  // Iterative loop
  auto_ptr next = this->iterate(); // First iteration happens here
  while(next->iterations() < params_->maxIterations &&
      !checkConvergence(params_->relativeErrorTol, params_->absoluteErrorTol,
          params_->errorTol, currentError, next->error(), params_->verbosity)) {

    // Do next iteration
    currentError = next->error();
    next = next->iterate();
  }

  // Printing if verbose
  if (params_->verbosity >= NonlinearOptimizerParams::VALUES)
    next->values()->print("final values");
  if (params_->verbosity >= NonlinearOptimizerParams::ERROR &&
      next->iterations() >= params_->maxIterations)
    cout << "Terminating because reached maximum iterations" << endl;
  if (params_->verbosity >= NonlinearOptimizerParams::ERROR)
    cout << "final error: " << next->error() << endl;

  // Return optimizer from final iteration
  return next;
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
		if (absoluteDecrease < absoluteErrorTreshold)
			cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " < " << absoluteErrorTreshold << endl;
		else
			cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " >= " << absoluteErrorTreshold << endl;
	}

	// calculate relative error decrease and update currentError
	double relativeDecrease = absoluteDecrease / currentError;
	if (verbosity >= 2) {
		if (relativeDecrease < relativeErrorTreshold)
			cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " < " << relativeErrorTreshold << endl;
		else
			cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " >= " << relativeErrorTreshold << endl;
	}
	bool converged = (relativeErrorTreshold && (relativeDecrease < relativeErrorTreshold))
			|| (absoluteDecrease < absoluteErrorTreshold);
	if (verbosity >= 1 && converged) {
		if(absoluteDecrease >= 0.0)
		  cout << "converged" << endl;
		else
		  cout << "Warning:  stopping nonlinear iterations because error increased" << endl;
	}
	return converged;
}


}
