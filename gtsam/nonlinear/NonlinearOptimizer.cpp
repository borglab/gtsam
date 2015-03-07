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
#include <boost/algorithm/string.hpp>
using namespace std;

namespace gtsam {

/* ************************************************************************* */
NonlinearOptimizerParams::Verbosity NonlinearOptimizerParams::verbosityTranslator(const std::string &src) const {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "SILENT") return NonlinearOptimizerParams::SILENT;
  if (s == "ERROR") return NonlinearOptimizerParams::ERROR;
  if (s == "VALUES") return NonlinearOptimizerParams::VALUES;
  if (s == "DELTA") return NonlinearOptimizerParams::DELTA;
  if (s == "LINEAR") return NonlinearOptimizerParams::LINEAR;

  /* default is silent */
  return NonlinearOptimizerParams::SILENT;
}

/* ************************************************************************* */
std::string NonlinearOptimizerParams::verbosityTranslator(Verbosity value) const {
  std::string s;
  switch (value) {
  case NonlinearOptimizerParams::SILENT:  s = "SILENT"; break;
  case NonlinearOptimizerParams::ERROR:   s = "ERROR"; break;
  case NonlinearOptimizerParams::VALUES:  s = "VALUES"; break;
  case NonlinearOptimizerParams::DELTA:   s = "DELTA"; break;
  case NonlinearOptimizerParams::LINEAR:  s = "LINEAR"; break;
  default:                                s = "UNDEFINED"; break;
  }
  return s;
}

/* ************************************************************************* */
void NonlinearOptimizerParams::print(const std::string& str) const {
  std::cout << str << "\n";
  std::cout << "relative decrease threshold: " << relativeErrorTol << "\n";
  std::cout << "absolute decrease threshold: " << absoluteErrorTol << "\n";
  std::cout << "      total error threshold: " << errorTol << "\n";
  std::cout << "         maximum iterations: " << maxIterations << "\n";
  std::cout << "                  verbosity: " << verbosityTranslator(verbosity) << "\n";
  std::cout.flush();
}

/* ************************************************************************* */
void NonlinearOptimizer::defaultOptimize() {

  const NonlinearOptimizerParams& params = this->_params();
  double currentError = this->error();

  // check if we're already close enough
  if(currentError <= params.errorTol) {
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params.errorTol << endl;
    return;
  }

  // Maybe show output
  if (params.verbosity >= NonlinearOptimizerParams::VALUES) this->values().print("Initial values");
  if (params.verbosity >= NonlinearOptimizerParams::ERROR) cout << "Initial error: " << currentError << endl;

  // Return if we already have too many iterations
  if(this->iterations() >= params.maxIterations)
    return;

  // Iterative loop
  do {
    // Do next iteration
    currentError = this->error();
    this->iterate();

    // Maybe show output
    if(params.verbosity >= NonlinearOptimizerParams::VALUES) this->values().print("newValues");
    if(params.verbosity >= NonlinearOptimizerParams::ERROR) cout << "newError: " << this->error() << endl;
  } while(this->iterations() < params.maxIterations &&
      !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol,
            params.errorTol, currentError, this->error(), params.verbosity));

  // Printing if verbose
  if (params.verbosity >= NonlinearOptimizerParams::ERROR &&
      this->iterations() >= params.maxIterations)
    cout << "Terminating because reached maximum iterations" << endl;
}

/* ************************************************************************* */
const Values& NonlinearOptimizer::optimizeSafely() {
	static const Values empty;
	try {
		defaultOptimize();
		return values();
	} catch (...) {
		// uncaught exception, returning empty result
		return empty;
	}
}

/* ************************************************************************* */
bool checkConvergence(double relativeErrorTreshold, double absoluteErrorTreshold,
    double errorThreshold, double currentError, double newError,
    NonlinearOptimizerParams::Verbosity verbosity) {

	if ( verbosity >= NonlinearOptimizerParams::ERROR ) {
		if ( newError <= errorThreshold )
			cout << "errorThreshold: " << newError << " < " << errorThreshold << endl;
		else
			cout << "errorThreshold: " << newError << " > " << errorThreshold << endl;
	}

	if ( newError <= errorThreshold ) return true ;

	// check if diverges
	double absoluteDecrease = currentError - newError;
	if (verbosity >= NonlinearOptimizerParams::ERROR) {
		if (absoluteDecrease <= absoluteErrorTreshold)
			cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " < " << absoluteErrorTreshold << endl;
		else
			cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " >= " << absoluteErrorTreshold << endl;
	}

	// calculate relative error decrease and update currentError
	double relativeDecrease = absoluteDecrease / currentError;
	if (verbosity >= NonlinearOptimizerParams::ERROR) {
		if (relativeDecrease <= relativeErrorTreshold)
			cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " < " << relativeErrorTreshold << endl;
		else
			cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " >= " << relativeErrorTreshold << endl;
	}
	bool converged = (relativeErrorTreshold && (relativeDecrease <= relativeErrorTreshold))
			|| (absoluteDecrease <= absoluteErrorTreshold);
	if (verbosity >= NonlinearOptimizerParams::ERROR && converged) {
		if(absoluteDecrease >= 0.0)
		  cout << "converged" << endl;
		else
		  cout << "Warning:  stopping nonlinear iterations because error increased" << endl;
	}
	return converged;
}
/* ************************************************************************* */


}
