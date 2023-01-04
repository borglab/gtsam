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
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>
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
// NOTE(frank): unique_ptr by-value takes ownership, as discussed in
// http://stackoverflow.com/questions/8114276/
NonlinearOptimizer::NonlinearOptimizer(const NonlinearFactorGraph& graph,
                                       std::unique_ptr<internal::NonlinearOptimizerState> state)
    : graph_(graph), state_(std::move(state)) {}

/* ************************************************************************* */
NonlinearOptimizer::~NonlinearOptimizer() {}

/* ************************************************************************* */
double NonlinearOptimizer::error() const {
  return state_->error;
}

size_t NonlinearOptimizer::iterations() const {
  return state_->iterations;
}

const Values& NonlinearOptimizer::values() const {
  return state_->values;
}

/* ************************************************************************* */
void NonlinearOptimizer::defaultOptimize() {
  const NonlinearOptimizerParams& params = _params();
  double currentError = error();

  // check if we're already close enough
  if (currentError <= params.errorTol) {
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params.errorTol << endl;
    return;
  }

  // Maybe show output
  if (params.verbosity >= NonlinearOptimizerParams::VALUES)
    values().print("Initial values");
  if (params.verbosity >= NonlinearOptimizerParams::ERROR)
    cout << "Initial error: " << currentError << endl;

  // Return if we already have too many iterations
  if (iterations() >= params.maxIterations) {
    if (params.verbosity >= NonlinearOptimizerParams::TERMINATION) {
      cout << "iterations: " << iterations() << " >? " << params.maxIterations << endl;
    }
    return;
  }

  // Iterative loop
  double newError = currentError; // used to avoid repeated calls to error()
  do {
    // Do next iteration
    currentError = newError;
    iterate();
    tictoc_finishedIteration();

    // Update newError for either printouts or conditional-end checks:
    newError = error();

    // User hook:
    if (params.iterationHook)
      params.iterationHook(iterations(), currentError, newError);

    // Maybe show output
    if (params.verbosity >= NonlinearOptimizerParams::VALUES)
      values().print("newValues");
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "newError: " << newError << endl;
  } while (iterations() < params.maxIterations &&
           !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol,
                             currentError, newError, params.verbosity) && std::isfinite(currentError));

  // Printing if verbose
  if (params.verbosity >= NonlinearOptimizerParams::TERMINATION) {
    cout << "iterations: " << iterations() << " >? " << params.maxIterations << endl;
    if (iterations() >= params.maxIterations)
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
VectorValues NonlinearOptimizer::solve(const GaussianFactorGraph& gfg,
                                       const NonlinearOptimizerParams& params) const {
  // solution of linear solver is an update to the linearization point
  VectorValues delta;

  // Check which solver we are using
  if (params.isMultifrontal()) {
    // Multifrontal QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.optimize(*params.ordering, params.getEliminationFunction());
    else
      delta = gfg.optimize(params.getEliminationFunction());
  } else if (params.isSequential()) {
    // Sequential QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.eliminateSequential(*params.ordering,
                                      params.getEliminationFunction())
                  ->optimize();
    else
      delta = gfg.eliminateSequential(params.orderingType,
                                      params.getEliminationFunction())
                  ->optimize();
  } else if (params.isIterative()) {
    // Conjugate Gradient -> needs params.iterativeParams
    if (!params.iterativeParams)
      throw std::runtime_error(
          "NonlinearOptimizer::solve: cg parameter has to be assigned ...");

    if (auto pcg = boost::dynamic_pointer_cast<PCGSolverParameters>(
            params.iterativeParams)) {
      delta = PCGSolver(*pcg).optimize(gfg);
    } else if (auto spcg =
                   boost::dynamic_pointer_cast<SubgraphSolverParameters>(
                       params.iterativeParams)) {
      if (!params.ordering)
        throw std::runtime_error("SubgraphSolver needs an ordering");
      delta = SubgraphSolver(gfg, *spcg, *params.ordering).optimize();
    } else {
      throw std::runtime_error(
          "NonlinearOptimizer::solve: special cg parameter type is not handled in LM solver ...");
    }
  } else {
    throw std::runtime_error("NonlinearOptimizer::solve: Optimization parameter is invalid");
  }

  // return update
  return delta;
}

/* ************************************************************************* */
bool checkConvergence(double relativeErrorTreshold, double absoluteErrorTreshold,
                      double errorThreshold, double currentError, double newError,
                      NonlinearOptimizerParams::Verbosity verbosity) {
  if (verbosity >= NonlinearOptimizerParams::ERROR) {
    if (newError <= errorThreshold)
      cout << "errorThreshold: " << newError << " < " << errorThreshold << endl;
    else
      cout << "errorThreshold: " << newError << " > " << errorThreshold << endl;
  }

  if (newError <= errorThreshold)
    return true;

  // check if diverges
  double absoluteDecrease = currentError - newError;
  if (verbosity >= NonlinearOptimizerParams::ERROR) {
    if (absoluteDecrease <= absoluteErrorTreshold)
      cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " < "
           << absoluteErrorTreshold << endl;
    else
      cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease
           << " >= " << absoluteErrorTreshold << endl;
  }

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / currentError;
  if (verbosity >= NonlinearOptimizerParams::ERROR) {
    if (relativeDecrease <= relativeErrorTreshold)
      cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " < "
           << relativeErrorTreshold << endl;
    else
      cout << "relativeDecrease: " << setprecision(12) << relativeDecrease
           << " >= " << relativeErrorTreshold << endl;
  }
  bool converged = (relativeErrorTreshold && (relativeDecrease <= relativeErrorTreshold)) ||
                   (absoluteDecrease <= absoluteErrorTreshold);
  if (verbosity >= NonlinearOptimizerParams::TERMINATION && converged) {
    if (absoluteDecrease >= 0.0)
      cout << "converged" << endl;
    else
      cout << "Warning:  stopping nonlinear iterations because error increased" << endl;

    cout << "errorThreshold: " << newError << " <? " << errorThreshold << endl;
    cout << "absoluteDecrease: " << setprecision(12) << absoluteDecrease << " <? "
         << absoluteErrorTreshold << endl;
    cout << "relativeDecrease: " << setprecision(12) << relativeDecrease << " <? "
         << relativeErrorTreshold << endl;
  }
  return converged;
}

/* ************************************************************************* */
GTSAM_EXPORT bool checkConvergence(const NonlinearOptimizerParams& params, double currentError,
                                   double newError) {
  return checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol,
                          currentError, newError, params.verbosity);
}
}
