/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussNewtonOptimizer.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Feb 26, 2012
 */

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

using namespace std;

namespace gtsam {

NonlinearOptimizer::auto_ptr GaussNewtonOptimizer::iterate() const {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_->linearize(values_, gnParams_->ordering);

  // Check whether to use QR
  const bool useQR;
  if(gnParams_->factorization == GaussNewtonParams::LDL)
    useQR = false;
  else if(gnParams_->factorization == GaussNewtonParams::QR)
    useQR = true;
  else
    throw runtime_error("Optimization parameter is invalid: GaussNewtonParams::factorization");

  // Optimize
  VectorValues::shared_ptr delta;
  if(gnParams_->elimination == MULTIFRONTAL)
    delta = GaussianMultifrontalSolver(*linear, useQR).optimize();
  else if(gnParams_->elimination == SEQUENTIAL)
    delta = GaussianSequentialSolver(*linear, useQR).optimize();
  else
    throw runtime_error("Optimization parameter is invalid: GaussNewtonParams::elimination");

  // Maybe show output
  if(params_->verbosity >= NonlinearOptimizerParams::DELTA) delta->print("delta");

  // Update values
  SharedValues newValues(new Values(values_->retract(*delta, gnParams_->ordering)));
  double newError = graph_->error(newValues);

  // Maybe show output
  if (params_->verbosity >= NonlinearOptimizerParams::VALUES) newValues->print("newValues");
  if (params_->verbosity >= NonlinearOptimizerParams::ERROR) cout << "error: " << newError << endl;

  // Create new optimizer with new values and new error
  auto_ptr<GaussNewtonOptimizer> newOptimizer(new GaussNewtonOptimizer(
      graph_, newValues, gnParams_, newError, iterations_+1));

  return newOptimizer;
}

} /* namespace gtsam */
