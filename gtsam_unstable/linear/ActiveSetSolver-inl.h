/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     ActiveSetSolver-inl.h
 * @brief    Implmentation of ActiveSetSolver.
 * @author   Ivan Dario Jimenez
 * @author   Duy Nguyen Ta
 * @date     2/11/16
 */

#pragma once

#include <gtsam_unstable/linear/InfeasibleInitialValues.h>

/******************************************************************************/
// Convenient macros to reduce syntactic noise. undef later.
#define Template template <class PROBLEM, class POLICY, class INITSOLVER>
#define This ActiveSetSolver<PROBLEM, POLICY, INITSOLVER>

/******************************************************************************/

namespace gtsam {

/* We have to make sure the new solution with alpha satisfies all INACTIVE inequality constraints
 * If some inactive inequality constraints complain about the full step (alpha = 1),
 * we have to adjust alpha to stay within the inequality constraints' feasible regions.
 *
 * For each inactive inequality j:
 *  - We already have: aj'*xk - bj <= 0, since xk satisfies all inequality constraints
 *  - We want: aj'*(xk + alpha*p) - bj <= 0
 *  - If aj'*p <= 0, we have: aj'*(xk + alpha*p) <= aj'*xk <= bj, for all alpha>0
 *  it's good!
 *  - We only care when aj'*p > 0. In this case, we need to choose alpha so that
 *  aj'*xk + alpha*aj'*p - bj <= 0  --> alpha <= (bj - aj'*xk) / (aj'*p)
 *  We want to step as far as possible, so we should choose alpha = (bj - aj'*xk) / (aj'*p)
 *
 * We want the minimum of all those alphas among all inactive inequality.
 */
Template std::tuple<double, int> This::computeStepSize(
    const InequalityFactorGraph& workingSet, const VectorValues& xk,
    const VectorValues& p, const double& maxAlpha) const {
  double minAlpha = maxAlpha;
  int closestFactorIx = -1;
  for (size_t factorIx = 0; factorIx < workingSet.size(); ++factorIx) {
    const LinearInequality::shared_ptr& factor = workingSet.at(factorIx);
    double b = factor->getb()[0];
    // only check inactive factors
    if (!factor->active()) {
      // Compute a'*p
      double aTp = factor->dotProductRow(p);

      // Check if  a'*p >0. Don't care if it's not.
      if (aTp <= 0)
        continue;

      // Compute a'*xk
      double aTx = factor->dotProductRow(xk);

      // alpha = (b - a'*xk) / (a'*p)
      double alpha = (b - aTx) / aTp;
      // We want the minimum of all those max alphas
      if (alpha < minAlpha) {
        closestFactorIx = factorIx;
        minAlpha = alpha;
      }
    }
  }
  return std::make_tuple(minAlpha, closestFactorIx);
}

/******************************************************************************/
/*
 * The goal of this function is to find currently active inequality constraints
 * that violate the condition to be active. The one that violates the condition
 * the most will be removed from the active set. See Nocedal06book, pg 469-471
 *
 * Find the BAD active inequality that pulls x strongest to the wrong direction
 * of its constraint (i.e. it is pulling towards >0, while its feasible region is <=0)
 *
 * For active inequality constraints (those that are enforced as equality constraints
 * in the current working set), we want lambda < 0.
 * This is because:
 *   - From the Lagrangian L = f - lambda*c, we know that the constraint force
 *     is (lambda * \grad c) = \grad f. Intuitively, to keep the solution x stay
 *     on the constraint surface, the constraint force has to balance out with
 *     other unconstrained forces that are pulling x towards the unconstrained
 *     minimum point. The other unconstrained forces are pulling x toward (-\grad f),
 *     hence the constraint force has to be exactly \grad f, so that the total
 *     force is 0.
 *   - We also know that  at the constraint surface c(x)=0, \grad c points towards + (>= 0),
 *     while we are solving for - (<=0) constraint.
 *   - We want the constraint force (lambda * \grad c) to pull x towards the - (<=0) direction
 *     i.e., the opposite direction of \grad c where the inequality constraint <=0 is satisfied.
 *     That means we want lambda < 0.
 *   - This is because when the constrained force pulls x towards the infeasible region (+),
 *     the unconstrained force is pulling x towards the opposite direction into
 *     the feasible region (again because the total force has to be 0 to make x stay still)
 *     So we can drop this constraint to have a lower error but feasible solution.
 *
 * In short, active inequality constraints with lambda > 0 are BAD, because they
 * violate the condition to be active.
 *
 * And we want to remove the worst one with the largest lambda from the active set.
 *
 */
Template int This::identifyLeavingConstraint(
    const InequalityFactorGraph& workingSet,
    const VectorValues& lambdas) const {
  int worstFactorIx = -1;
  // preset the maxLambda to 0.0: if lambda is <= 0.0, the constraint is either
  // inactive or a good inequality constraint, so we don't care!
  double maxLambda = 0.0;
  for (size_t factorIx = 0; factorIx < workingSet.size(); ++factorIx) {
    const LinearInequality::shared_ptr& factor = workingSet.at(factorIx);
    if (factor->active()) {
      double lambda = lambdas.at(factor->dualKey())[0];
      if (lambda > maxLambda) {
        worstFactorIx = factorIx;
        maxLambda = lambda;
      }
    }
  }
  return worstFactorIx;
}

//******************************************************************************
Template JacobianFactor::shared_ptr This::createDualFactor(
    Key key, const InequalityFactorGraph& workingSet,
    const VectorValues& delta) const {
  // Transpose the A matrix of constrained factors to have the jacobian of the
  // dual key
  TermsContainer Aterms = collectDualJacobians<LinearEquality>(
      key, problem_.equalities, equalityVariableIndex_);
  TermsContainer AtermsInequalities = collectDualJacobians<LinearInequality>(
      key, workingSet, inequalityVariableIndex_);
  Aterms.insert(Aterms.end(), AtermsInequalities.begin(),
                AtermsInequalities.end());

  // Collect the gradients of unconstrained cost factors to the b vector
  if (Aterms.size() > 0) {
    Vector b = problem_.costGradient(key, delta);
    // to compute the least-square approximation of dual variables
    return std::make_shared<JacobianFactor>(Aterms, b);
  } else {
    return nullptr;
  }
}

/******************************************************************************/
/*  This function will create a dual graph that solves for the
 *  lagrange multipliers for the current working set.
 *  You can use lagrange multipliers as a necessary condition for optimality.
 *  The factor graph that is being solved is f' = -lambda * g'
 *  where f is the optimized function and g is the function resulting from
 *  aggregating the working set.
 *  The lambdas give you information about the feasibility of a constraint.
 *  if lambda < 0  the constraint is Ok
 *  if lambda = 0  you are on the constraint
 *  if lambda > 0  you are violating the constraint.
 */
Template GaussianFactorGraph This::buildDualGraph(
    const InequalityFactorGraph& workingSet, const VectorValues& delta) const {
  GaussianFactorGraph dualGraph;
  for (Key key : constrainedKeys_) {
    // Each constrained key becomes a factor in the dual graph
    auto dualFactor = createDualFactor(key, workingSet, delta);
    if (dualFactor) dualGraph.push_back(dualFactor);
  }
  return dualGraph;
}

//******************************************************************************
Template GaussianFactorGraph
This::buildWorkingGraph(const InequalityFactorGraph& workingSet,
                        const VectorValues& xk) const {
  GaussianFactorGraph workingGraph;
  workingGraph.push_back(POLICY::buildCostFunction(problem_, xk));
  workingGraph.push_back(problem_.equalities);
  for (const LinearInequality::shared_ptr& factor : workingSet)
    if (factor->active()) workingGraph.push_back(factor);
  return workingGraph;
}

//******************************************************************************
Template typename This::State This::iterate(
    const typename This::State& state) const {
  // Algorithm 16.3 from Nocedal06book.
  // Solve with the current working set eqn 16.39, but solve for x not p
  auto workingGraph = buildWorkingGraph(state.workingSet, state.values);
  VectorValues newValues = workingGraph.optimize();
  // If we CAN'T move further
  // if p_k = 0 is the original condition, modified by Duy to say that the state
  // update is zero.
  if (newValues.equals(state.values, 1e-7)) {
    // Compute lambda from the dual graph
    auto dualGraph = buildDualGraph(state.workingSet, newValues);
    VectorValues duals = dualGraph.optimize();
    int leavingFactor = identifyLeavingConstraint(state.workingSet, duals);
    // If all inequality constraints are satisfied: We have the solution!!
    if (leavingFactor < 0) {
      return State(newValues, duals, state.workingSet, true,
          state.iterations + 1);
    } else {
      // Inactivate the leaving constraint
      InequalityFactorGraph newWorkingSet = state.workingSet;
      newWorkingSet.at(leavingFactor)->inactivate();
      return State(newValues, duals, newWorkingSet, false,
          state.iterations + 1);
    }
  } else {
    // If we CAN make some progress, i.e. p_k != 0
    // Adapt stepsize if some inactive constraints complain about this move
    VectorValues p = newValues - state.values;
    const auto [alpha, factorIx] = // using 16.41
        computeStepSize(state.workingSet, state.values, p, POLICY::maxAlpha);
    // also add to the working set the one that complains the most
    InequalityFactorGraph newWorkingSet = state.workingSet;
    if (factorIx >= 0)
      newWorkingSet.at(factorIx)->activate();
    // step!
    newValues = state.values + alpha * p;
    return State(newValues, state.duals, newWorkingSet, false,
        state.iterations + 1);
  }
}

//******************************************************************************
Template InequalityFactorGraph This::identifyActiveConstraints(
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
      if (std::abs(error) < 1e-7)
        workingFactor->activate();
      else
        workingFactor->inactivate();
    }
    workingSet.push_back(workingFactor);
  }
  return workingSet;
}

//******************************************************************************
Template std::pair<VectorValues, VectorValues> This::optimize(
    const VectorValues& initialValues, const VectorValues& duals,
    bool useWarmStart) const {
  // Initialize workingSet from the feasible initialValues
  InequalityFactorGraph workingSet = identifyActiveConstraints(
      problem_.inequalities, initialValues, duals, useWarmStart);
  State state(initialValues, duals, workingSet, false, 0);

  /// main loop of the solver
  while (!state.converged) state = iterate(state);

  return std::make_pair(state.values, state.duals);
}

//******************************************************************************
Template std::pair<VectorValues, VectorValues> This::optimize() const {
  INITSOLVER initSolver(problem_);
  VectorValues initValues = initSolver.solve();
  return optimize(initValues);
}

}

#undef Template
#undef This
