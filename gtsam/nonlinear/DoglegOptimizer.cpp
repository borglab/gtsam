/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DoglegOptimizer.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Feb 26, 2012
 */

#include <gtsam/nonlinear/DoglegOptimizer.h>

#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
NonlinearOptimizer::SharedState DoglegOptimizer::iterate(const NonlinearOptimizer::SharedState& current) const {

  // Linearize graph
  const Ordering& ordering = *ordering(current->values);
  GaussianFactorGraph::shared_ptr linear = graph_->linearize(current->values, ordering);

  // Check whether to use QR
  bool useQR;
  if(params_->factorization == DoglegParams::LDL)
    useQR = false;
  else if(params_->factorization == DoglegParams::QR)
    useQR = true;
  else
    throw runtime_error("Optimization parameter is invalid: DoglegParams::factorization");

  // Pull out parameters we'll use
  const bool dlVerbose = (params_->dlVerbosity > DoglegParams::SILENT);

  // Do Dogleg iteration with either Multifrontal or Sequential elimination
  DoglegOptimizerImpl::IterationResult result;

  if(params_->elimination == DoglegParams::MULTIFRONTAL) {
    GaussianBayesTree::shared_ptr bt = GaussianMultifrontalSolver(*linear, useQR).eliminate();
    result = DoglegOptimizerImpl::Iterate(current->Delta, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, *bt, *graph_, *values(), ordering, error(), dlVerbose);

  } else if(params_->elimination == DoglegParams::SEQUENTIAL) {
    GaussianBayesNet::shared_ptr bn = GaussianSequentialSolver(*linear, useQR).eliminate();
    result = DoglegOptimizerImpl::Iterate(current->Delta, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, *bn, *graph_, *values(), ordering, error(), dlVerbose);

  } else {
    throw runtime_error("Optimization parameter is invalid: DoglegParams::elimination");
  }

  // Create new state with new values and new error
  DoglegOptimizer::SharedState newState = boost::make_shared<DoglegState>();

  // Update values
  newState->values = current->values.retract(result.dx_d, ordering);

  newState->error = result.f_error;
  newState->iterations = current->iterations + 1;
  newState->Delta = result.Delta;

  return newState;
}

/* ************************************************************************* */
NonlinearOptimizer::SharedState DoglegOptimizer::initialState(const Values& initialValues) const {
  SharedState initial = boost::make_shared<DoglegState>();
  defaultInitialState(initialValues);
  initial->Delta = params_->deltaInitial;
  return initial;
}

} /* namespace gtsam */
