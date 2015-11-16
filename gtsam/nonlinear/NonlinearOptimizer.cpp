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

#include <gtsam/nonlinear/NonlinearOptimizer.h>

#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <gtsam/inference/Ordering.h>

#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>

#include <stdexcept>
#include <iostream>
#include <iomanip>

using namespace std;

namespace gtsam {

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
  if(this->iterations() >= params.maxIterations){
    if (params.verbosity >= NonlinearOptimizerParams::TERMINATION) {
        cout << "iterations: " << this->iterations() << " >? " << params.maxIterations << endl;
      }
    return;
  }

  // Iterative loop
  do {
    // Do next iteration
    currentError = this->error();
    this->iterate();
    tictoc_finishedIteration();

    // Maybe show output
    if(params.verbosity >= NonlinearOptimizerParams::VALUES) this->values().print("newValues");
    if(params.verbosity >= NonlinearOptimizerParams::ERROR) cout << "newError: " << this->error() << endl;
  } while(this->iterations() < params.maxIterations &&
      !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol,
            params.errorTol, currentError, this->error(), params.verbosity));

  // Printing if verbose
  if (params.verbosity >= NonlinearOptimizerParams::TERMINATION) {
    cout << "iterations: " << this->iterations() << " >? " << params.maxIterations << endl;
    if (this->iterations() >= params.maxIterations)
      cout << "Terminating because reached maximum iterations" << endl;
  }
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
VectorValues NonlinearOptimizer::solve(const GaussianFactorGraph &gfg,
    const Values& initial, const NonlinearOptimizerParams& params) const {

  // solution of linear solver is an update to the linearization point
  VectorValues delta;

  // Check which solver we are using
  if (params.isMultifrontal()) {
    // Multifrontal QR or Cholesky (decided by params.getEliminationFunction())
    delta = gfg.optimize(*params.ordering, params.getEliminationFunction());
  } else if (params.isSequential()) {
    // Sequential QR or Cholesky (decided by params.getEliminationFunction())
    delta = gfg.eliminateSequential(*params.ordering, params.getEliminationFunction(), 
								     boost::none, params.orderingType)->optimize();
  } else if (params.isIterative()) {

    // Conjugate Gradient -> needs params.iterativeParams
    if (!params.iterativeParams)
      throw std::runtime_error("NonlinearOptimizer::solve: cg parameter has to be assigned ...");

    if (boost::shared_ptr<PCGSolverParameters> pcg = boost::dynamic_pointer_cast<PCGSolverParameters>(params.iterativeParams) ) {
      delta = PCGSolver(*pcg).optimize(gfg);
    }
    else if (boost::shared_ptr<SubgraphSolverParameters> spcg = boost::dynamic_pointer_cast<SubgraphSolverParameters>(params.iterativeParams) ) {
      delta = SubgraphSolver(gfg, *spcg, *params.ordering).optimize();
    }
    else {
      throw std::runtime_error("NonlinearOptimizer::solve: special cg parameter type is not handled in LM solver ...");
    }
  } else {
    throw std::runtime_error(
        "NonlinearOptimizer::solve: Optimization parameter is invalid");
  }

  // return update
  return delta;
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
  if (verbosity >= NonlinearOptimizerParams::TERMINATION && converged) {

    if(absoluteDecrease >= 0.0)
      cout << "converged" << endl;
    else
      cout << "Warning:  stopping nonlinear iterations because error increased" << endl;

    cout << "errorThreshold: " << newError << " <? " << errorThreshold << endl;
    cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " <? " << absoluteErrorTreshold << endl;
    cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " <? " << relativeErrorTreshold << endl;
  }
  return converged;
}

/* ************************************************************************* */


}
