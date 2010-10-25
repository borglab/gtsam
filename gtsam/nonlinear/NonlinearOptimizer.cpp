/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * NonlinearOptimizer.cpp
 * @brief: Dummy c++ file to have *something* in lin-nonlinear .
 * @Author: Frank Dellaert
 * Created on: Jul 17, 2010
 */

#include <iostream>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

using namespace std;

namespace gtsam {


bool check_convergence (
		const NonlinearOptimizationParameters &parameters,
		double currentError, double newError) {
	return check_convergence(parameters.relDecrease_,
							 parameters.absDecrease_,
							 parameters.sumError_,
							 currentError, newError,
							 parameters.verbosity_) ;
}

bool check_convergence(
		double relativeErrorTreshold,
		double absoluteErrorTreshold,
		double errorThreshold,
		double currentError, double newError, int verbosity) {

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
			cout << "absoluteDecrease: " << absoluteDecrease << " < " << absoluteErrorTreshold << endl;
		else
			cout << "absoluteDecrease: " << absoluteDecrease << " >= " << absoluteErrorTreshold << endl;
	}

	// calculate relative error decrease and update currentError
	double relativeDecrease = absoluteDecrease / currentError;
	if (verbosity >= 2) {
		if (relativeDecrease < relativeErrorTreshold)
			cout << "relativeDecrease: " << relativeDecrease << " < " << relativeErrorTreshold << endl;
		else
			cout << "relativeDecrease: " << relativeDecrease << " >= " << relativeErrorTreshold << endl;
	}
	bool converged = (relativeDecrease < relativeErrorTreshold)
			|| (absoluteDecrease < absoluteErrorTreshold);
	if (verbosity >= 1 && converged)
		cout << "converged" << endl;
	return converged;
}


}
