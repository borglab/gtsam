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

NonlinearOptimizer::auto_ptr DoglegOptimizer::iterate() const {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_->linearize(*values_, *ordering_);

  // Check whether to use QR
  bool useQR;
  if(dlParams_->factorization == DoglegParams::LDL)
    useQR = false;
  else if(dlParams_->factorization == DoglegParams::QR)
    useQR = true;
  else
    throw runtime_error("Optimization parameter is invalid: DoglegParams::factorization");

  // Pull out parameters we'll use
  const bool dlVerbose = (dlParams_->dlVerbosity > DoglegParams::SILENT);

  // Do Dogleg iteration with either Multifrontal or Sequential elimination
  DoglegOptimizerImpl::IterationResult result;

  if(dlParams_->elimination == DoglegParams::MULTIFRONTAL) {
    GaussianBayesTree::shared_ptr bt = GaussianMultifrontalSolver(*linear, useQR).eliminate();
    result = DoglegOptimizerImpl::Iterate(delta_, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, *bt, *graph_, *values(), *ordering_, error(), dlVerbose);

  } else if(dlParams_->elimination == DoglegParams::SEQUENTIAL) {
    GaussianBayesNet::shared_ptr bn = GaussianSequentialSolver(*linear, useQR).eliminate();
    result = DoglegOptimizerImpl::Iterate(delta_, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, *bn, *graph_, *values(), *ordering_, error(), dlVerbose);

  } else {
    throw runtime_error("Optimization parameter is invalid: DoglegParams::elimination");
  }

  // Update values
  SharedValues newValues(new Values(values_->retract(result.dx_d, *ordering_)));

  // Create new optimizer with new values and new error
  NonlinearOptimizer::auto_ptr newOptimizer(new DoglegOptimizer(
      *this, newValues, result.f_error, result.Delta));

  return newOptimizer;
}

} /* namespace gtsam */
