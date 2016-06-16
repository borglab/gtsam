/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QPSolver.cpp
 * @brief
 * @date    Apr 15, 2014
 * @author  Duy-Nguyen Ta
 */

#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/linear/QPInitSolver.h>
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>

using namespace std;

namespace gtsam {

//******************************************************************************
QPSolver::QPSolver(const QP& qp) :
    qp_(qp) {
  equalityVariableIndex_ = VariableIndex(qp_.equalities);
  inequalityVariableIndex_ = VariableIndex(qp_.inequalities);
  constrainedKeys_ = qp_.equalities.keys();
  constrainedKeys_.merge(qp_.inequalities.keys());
}

//******************************************************************************
GaussianFactorGraph QPSolver::buildWorkingGraph(
    const InequalityFactorGraph& workingSet, const VectorValues& xk) const {
  GaussianFactorGraph workingGraph;
  workingGraph.push_back(buildCostFunction(xk));
  workingGraph.push_back(qp_.equalities);
  for (const LinearInequality::shared_ptr& factor : workingSet) {
    if (factor->active()) workingGraph.push_back(factor);
  }
  return workingGraph;
}

//******************************************************************************
JacobianFactor::shared_ptr QPSolver::createDualFactor(
    Key key, const InequalityFactorGraph& workingSet,
    const VectorValues& delta) const {
  // Transpose the A matrix of constrained factors to have the jacobian of the
  // dual key
  TermsContainer Aterms = collectDualJacobians<LinearEquality>(
      key, qp_.equalities, equalityVariableIndex_);
  TermsContainer AtermsInequalities = collectDualJacobians<LinearInequality>(
      key, workingSet, inequalityVariableIndex_);
  Aterms.insert(Aterms.end(), AtermsInequalities.begin(),
                AtermsInequalities.end());

  // Collect the gradients of unconstrained cost factors to the b vector
  if (Aterms.size() > 0) {
    Vector b = qp_.costGradient(key, delta);
    // to compute the least-square approximation of dual variables
    return boost::make_shared<JacobianFactor>(Aterms, b);
  } else {
    return boost::make_shared<JacobianFactor>();
  }
}

//******************************************************************************
boost::tuple<double, int> QPSolver::computeStepSize(
    const InequalityFactorGraph& workingSet, const VectorValues& xk,
    const VectorValues& p) const {
  return ActiveSetSolver::computeStepSize(workingSet, xk, p, 1);
}

//******************************************************************************
QPState QPSolver::iterate(const QPState& state) const {
  // Algorithm 16.3 from Nocedal06book.
  // Solve with the current working set eqn 16.39, but instead of solving for p
  // solve for x
  GaussianFactorGraph workingGraph =
      buildWorkingGraph(state.workingSet, state.values);
  VectorValues newValues = workingGraph.optimize();
  // If we CAN'T move further
  // if p_k = 0 is the original condition, modified by Duy to say that the state
  // update is zero.
  if (newValues.equals(state.values, 1e-7)) {
    // Compute lambda from the dual graph
    GaussianFactorGraph::shared_ptr dualGraph = buildDualGraph(state.workingSet,
        newValues);
    VectorValues duals = dualGraph->optimize();
    int leavingFactor = identifyLeavingConstraint(state.workingSet, duals);
    // If all inequality constraints are satisfied: We have the solution!!
    if (leavingFactor < 0) {
      return QPState(newValues, duals, state.workingSet, true,
          state.iterations + 1);
    } else {
      // Inactivate the leaving constraint
      InequalityFactorGraph newWorkingSet = state.workingSet;
      newWorkingSet.at(leavingFactor)->inactivate();
      return QPState(newValues, duals, newWorkingSet, false,
          state.iterations + 1);
    }
  } else {
    // If we CAN make some progress, i.e. p_k != 0
    // Adapt stepsize if some inactive constraints complain about this move
    double alpha;
    int factorIx;
    VectorValues p = newValues - state.values;
    boost::tie(alpha, factorIx) = // using 16.41
        computeStepSize(state.workingSet, state.values, p);
    // also add to the working set the one that complains the most
    InequalityFactorGraph newWorkingSet = state.workingSet;
    if (factorIx >= 0)
      newWorkingSet.at(factorIx)->activate();
    // step!
    newValues = state.values + alpha * p;
    return QPState(newValues, state.duals, newWorkingSet, false,
        state.iterations + 1);
  }
}

//******************************************************************************
InequalityFactorGraph QPSolver::identifyActiveConstraints(
    const InequalityFactorGraph& inequalities,
    const VectorValues& initialValues, const VectorValues& duals,
    bool useWarmStart) const {
  InequalityFactorGraph workingSet;
  for (const LinearInequality::shared_ptr& factor : inequalities) {
    LinearInequality::shared_ptr workingFactor(new LinearInequality(*factor));
    if (useWarmStart && duals.size() > 0) {
      if (duals.exists(workingFactor->dualKey())) workingFactor->activate();
      else workingFactor->inactivate();
    } else {
      double error = workingFactor->error(initialValues);
      // Safety guard. This should not happen unless users provide a bad init
      if (error > 0) throw InfeasibleInitialValues();
      if (fabs(error) < 1e-7)
        workingFactor->activate();
      else
        workingFactor->inactivate();
    }
    workingSet.push_back(workingFactor);
  }
  return workingSet;
}

//******************************************************************************
pair<VectorValues, VectorValues> QPSolver::optimize(
    const VectorValues& initialValues, const VectorValues& duals,
    bool useWarmStart) const {
  // Initialize workingSet from the feasible initialValues
  InequalityFactorGraph workingSet = identifyActiveConstraints(qp_.inequalities,
      initialValues, duals, useWarmStart);
  QPState state(initialValues, duals, workingSet, false, 0);

  /// main loop of the solver
  while (!state.converged)
    state = iterate(state);

  return make_pair(state.values, state.duals);
}

//******************************************************************************
pair<VectorValues, VectorValues> QPSolver::optimize() const {
  QPInitSolver initSolver(qp_);
  VectorValues initValues = initSolver.solve();
  return optimize(initValues);
}

} /* namespace gtsam */
