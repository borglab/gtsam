/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SQPSimple.cpp
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#include <gtsam_unstable/nonlinear/SQPSimple.h>
#include <gtsam_unstable/linear/QPSolver.h>

namespace gtsam {


/* ************************************************************************* */
bool SQPSimple::isStationary(const VectorValues& delta) const {
  return delta.vector().lpNorm<Eigen::Infinity>() < errorTol;
}

/* ************************************************************************* */
bool SQPSimple::isPrimalFeasible(const SQPSimpleState& state) const {
  return nlp_.linearEqualities.checkFeasibility(state.values, errorTol)
      && nlp_.nonlinearEqualities.checkFeasibility(state.values, errorTol);
}

/* ************************************************************************* */
bool SQPSimple::isDualFeasible(const VectorValues& duals) const {
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.linearInequalities) {
    NonlinearConstraint::shared_ptr inequality = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
    Key dualKey = inequality->dualKey();
    if (!duals.exists(dualKey)) continue; // should be inactive constraint!
    double dual = duals.at(dualKey)[0];   // because we only support single-valued inequalities
    if (dual < 0.0)
      return false;
  }
  return true;
}

/* ************************************************************************* */
bool SQPSimple::isComplementary(const SQPSimpleState& state) const {
  return nlp_.linearInequalities.checkFeasibilityAndComplimentary(state.values, state.duals, errorTol);
}

/* ************************************************************************* */
bool SQPSimple::checkConvergence(const SQPSimpleState& state, const VectorValues& delta) const {
  return isStationary(delta) && isPrimalFeasible(state) && isDualFeasible(state.duals) && isComplementary(state);
}

/* ************************************************************************* */
VectorValues SQPSimple::initializeDuals() const {
  VectorValues duals;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.linearEqualities) {
    NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
    duals.insert(constraint->dualKey(), zero(factor->dim()));
  }

  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.nonlinearEqualities) {
    NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
    duals.insert(constraint->dualKey(), zero(factor->dim()));
  }
  return duals;
}

/* ************************************************************************* */
std::pair<Values, VectorValues> SQPSimple::optimize(const Values& initialValues) const {
  SQPSimpleState state(initialValues);
  state.duals = initializeDuals();
  while (!state.converged && state.iterations < 100) {
    state = iterate(state);
  }
  return std::make_pair(state.values, state.duals);
}

/* ************************************************************************* */
SQPSimpleState SQPSimple::iterate(const SQPSimpleState& state) const {
  static const bool debug = false;

  // construct the qp subproblem
  QP qp;
  qp.cost = *nlp_.cost.linearize(state.values);
  GaussianFactorGraph::shared_ptr multipliedHessians = nlp_.nonlinearEqualities.multipliedHessians(state.values, state.duals);
  qp.cost.push_back(*multipliedHessians);

  qp.equalities.add(*nlp_.linearEqualities.linearize(state.values));
  qp.equalities.add(*nlp_.nonlinearEqualities.linearize(state.values));

  qp.inequalities.add(*nlp_.linearInequalities.linearize(state.values));

  if (debug)
    qp.print("QP subproblem:");

  // solve the QP subproblem
  VectorValues delta, duals;
  QPSolver qpSolver(qp);
  boost::tie(delta, duals) = qpSolver.optimize();

  if (debug)
    delta.print("delta = ");

  if (debug)
    duals.print("duals = ");

  // update new state
  SQPSimpleState newState;
  newState.values = state.values.retract(delta);
  newState.duals = duals;
  newState.converged = checkConvergence(newState, delta);
  newState.iterations = state.iterations + 1;

  return newState;
}
}



