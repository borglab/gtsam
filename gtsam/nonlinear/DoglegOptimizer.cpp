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

#include <gtsam/inference/EliminationTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void DoglegOptimizer::iterate(void) {

  // Linearize graph
  const Ordering& ordering = *params_.ordering;
  GaussianFactorGraph::shared_ptr linear = graph_.linearize(state_.values, ordering);

  // Get elimination method
  GaussianFactorGraph::Eliminate eliminationMethod = params_.getEliminationFunction();

  // Pull out parameters we'll use
  const bool dlVerbose = (params_.verbosityDL > DoglegParams::SILENT);

  // Do Dogleg iteration with either Multifrontal or Sequential elimination
  DoglegOptimizerImpl::IterationResult result;

  if(params_.elimination == DoglegParams::MULTIFRONTAL) {
    GaussianBayesTree bt;
    bt.insert(GaussianJunctionTree(*linear).eliminate(eliminationMethod));
    result = DoglegOptimizerImpl::Iterate(state_.Delta, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, bt, graph_, state_.values, ordering, state_.error, dlVerbose);

  } else if(params_.elimination == DoglegParams::SEQUENTIAL) {
    GaussianBayesNet::shared_ptr bn = EliminationTree<GaussianFactor>::Create(*linear)->eliminate(eliminationMethod);
    result = DoglegOptimizerImpl::Iterate(state_.Delta, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, *bn, graph_, state_.values, ordering, state_.error, dlVerbose);

  } else {
    throw runtime_error("Optimization parameter is invalid: DoglegParams::elimination");
  }

  // Maybe show output
  if(params_.verbosity >= NonlinearOptimizerParams::DELTA) result.dx_d.print("delta");

  // Create new state with new values and new error
  state_.values = state_.values.retract(result.dx_d, ordering);
  state_.error = result.f_error;
  state_.Delta = result.Delta;
  ++state_.iterations;
}

} /* namespace gtsam */
