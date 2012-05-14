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

/* ************************************************************************* */
void GaussNewtonOptimizer::iterate() {

  const NonlinearOptimizerState& current = state_;

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_.linearize(current.values, *params_.ordering);

  // Check whether to use QR
  bool useQR;
  if(params_.factorization == GaussNewtonParams::LDL)
    useQR = false;
  else if(params_.factorization == GaussNewtonParams::QR)
    useQR = true;
  else
    throw runtime_error("Optimization parameter is invalid: GaussNewtonParams::factorization");

  // Optimize
  VectorValues::shared_ptr delta;
  if(params_.elimination == GaussNewtonParams::MULTIFRONTAL)
    delta = GaussianMultifrontalSolver(*linear, useQR).optimize();
  else if(params_.elimination == GaussNewtonParams::SEQUENTIAL)
    delta = GaussianSequentialSolver(*linear, useQR).optimize();
  else
    throw runtime_error("Optimization parameter is invalid: GaussNewtonParams::elimination");

  // Maybe show output
  if(params_.verbosity >= NonlinearOptimizerParams::DELTA) delta->print("delta");

  // Create new state with new values and new error
  state_.values = current.values.retract(*delta, *params_.ordering);
  state_.error = graph_.error(state_.values);
  ++ state_.iterations;
}

} /* namespace gtsam */
