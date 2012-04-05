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
NonlinearOptimizer::SharedState GaussNewtonOptimizer::iterate(const NonlinearOptimizer::SharedState& _current) const {

  const GaussNewtonState& current = dynamic_cast<const GaussNewtonState&>(*_current);

  // Linearize graph
  const Ordering& ordering = this->ordering(current.values);
  GaussianFactorGraph::shared_ptr linear = graph_->linearize(current.values, ordering);

  // Check whether to use QR
  bool useQR;
  if(params_->factorization == GaussNewtonParams::LDL)
    useQR = false;
  else if(params_->factorization == GaussNewtonParams::QR)
    useQR = true;
  else
    throw runtime_error("Optimization parameter is invalid: GaussNewtonParams::factorization");

  // Optimize
  VectorValues::shared_ptr delta;
  if(params_->elimination == GaussNewtonParams::MULTIFRONTAL)
    delta = GaussianMultifrontalSolver(*linear, useQR).optimize();
  else if(params_->elimination == GaussNewtonParams::SEQUENTIAL)
    delta = GaussianSequentialSolver(*linear, useQR).optimize();
  else
    throw runtime_error("Optimization parameter is invalid: GaussNewtonParams::elimination");

  // Maybe show output
  if(params_->verbosity >= NonlinearOptimizerParams::DELTA) delta->print("delta");

  // Create new state with new values and new error
  SharedState newState = boost::make_shared<GaussNewtonState>();
  newState->values = current.values.retract(*delta, ordering);
  newState->error = graph_->error(newState->values);
  newState->iterations = current.iterations + 1;

  return newState;
}

/* ************************************************************************* */
NonlinearOptimizer::SharedState GaussNewtonOptimizer::initialState(const Values& initialValues) const {
  SharedState initial = boost::make_shared<GaussNewtonState>();
  defaultInitialState(initialValues, *initial);
  return initial;
}

} /* namespace gtsam */
