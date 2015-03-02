/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearConstraintSQP.cpp
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/LinearConstraintSQP.h>
#include <gtsam_unstable/nonlinear/ConstrainedFactor.h>
#include <iostream>

namespace gtsam {

/* ************************************************************************* */
bool LinearConstraintSQP::isStationary(const VectorValues& delta) const {
  return delta.vector().lpNorm<Eigen::Infinity>() < params_.errorTol;
}

/* ************************************************************************* */
bool LinearConstraintSQP::isPrimalFeasible(const LinearConstraintNLPState& state) const {
  return lcnlp_.linearEqualities.checkFeasibility(state.values, params_.errorTol);
}

/* ************************************************************************* */
bool LinearConstraintSQP::isDualFeasible(const VectorValues& duals) const {
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, lcnlp_.linearInequalities) {
    ConstrainedFactor::shared_ptr inequality
        = boost::dynamic_pointer_cast<ConstrainedFactor>(factor);

    Key dualKey = inequality->dualKey();
    if (!duals.exists(dualKey)) continue; // should be inactive constraint!
    double dual = duals.at(dualKey)[0];// because we only support single-valued inequalities
    if (dual > 0.0) // See the explanation in QPSolver::identifyLeavingConstraint, we want dual < 0 ?
      return false;
  }
  return true;
}

/* ************************************************************************* */
bool LinearConstraintSQP::isComplementary(const LinearConstraintNLPState& state) const {
  return lcnlp_.linearInequalities.checkFeasibilityAndComplimentary(
      state.values, state.duals, params_.errorTol);
}

/* ************************************************************************* */
bool LinearConstraintSQP::checkConvergence(const LinearConstraintNLPState& state,
    const VectorValues& delta) const {
  return isStationary(delta) && isPrimalFeasible(state)
      && isDualFeasible(state.duals) && isComplementary(state);
}

/* ************************************************************************* */
VectorValues LinearConstraintSQP::initializeDuals() const {
  VectorValues duals;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, lcnlp_.linearEqualities){
    ConstrainedFactor::shared_ptr constraint
        = boost::dynamic_pointer_cast<ConstrainedFactor>(factor);
    duals.insert(constraint->dualKey(), zero(factor->dim()));
  }

  return duals;
}

/* ************************************************************************* */
LinearConstraintNLPState LinearConstraintSQP::iterate(
    const LinearConstraintNLPState& state) const {

  // construct the qp subproblem
  QP qp;
  qp.cost = *lcnlp_.cost.linearize(state.values);
  qp.equalities.add(*lcnlp_.linearEqualities.linearize(state.values));
  qp.inequalities.add(*lcnlp_.linearInequalities.linearize(state.values));

  if(params_.verbosity >= NonlinearOptimizerParams::LINEAR)
    qp.print("QP subproblem:");

  // solve the QP subproblem
  VectorValues delta, duals;
  QPSolver qpSolver(qp);
  VectorValues zeroInitialValues;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, state.values)
    zeroInitialValues.insert(key_value.key, zero(key_value.value.dim()));

  boost::tie(delta, duals) = qpSolver.optimize(zeroInitialValues, state.duals,
      params_.warmStart);

  if(params_.verbosity >= NonlinearOptimizerParams::DELTA)
    delta.print("Delta");

  // update new state
  LinearConstraintNLPState newState;
  newState.values = state.values.retract(delta);
  newState.duals = duals;
  newState.converged = checkConvergence(newState, delta);
  newState.iterations = state.iterations + 1;

  if(params_.verbosity >= NonlinearOptimizerParams::VALUES)
    newState.print("Values");

  return newState;
}

/* ************************************************************************* */
std::pair<Values, VectorValues> LinearConstraintSQP::optimize(
    const Values& initialValues) const {
  LinearConstraintNLPState state(initialValues);
  state.duals = initializeDuals();
  while (!state.converged && state.iterations < params_.maxIterations) {
    if(params_.verbosity >= NonlinearOptimizerParams::ERROR)
      std::cout << "Iteration # " << state.iterations << std::endl;
    state = iterate(state);
  }
  if(params_.verbosity >= NonlinearOptimizerParams::TERMINATION)
    std::cout << "Number of iterations: " << state.iterations << std::endl;
  return std::make_pair(state.values, state.duals);
}

} // namespace gtsam

