/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QPSolver.h
 * @brief   A quadratic programming solver implements the active set method
 * @date    Apr 15, 2014
 * @author  Ivan Dario Jimenez
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam_unstable/linear/QP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/QPState.h>
#include <gtsam/linear/VectorValues.h>

#include <vector>
#include <set>

namespace gtsam {

/**
 * This QPSolver uses the active set method to solve a quadratic programming problem
 * defined in the QP struct.
 * Note: This version of QPSolver only works with a feasible initial value.
 */
//TODO: Remove Vector Values
class QPSolver: public ActiveSetSolver {

  const QP& qp_; //!< factor graphs of the QP problem, can't be modified!

public:
  /// Constructor
  QPSolver(const QP& qp);

  /// Find solution with the current working set
  VectorValues solveWithCurrentWorkingSet(
      const InequalityFactorGraph& workingSet) const;

  /// Create a dual factor
  JacobianFactor::shared_ptr createDualFactor(Key key,
      const InequalityFactorGraph& workingSet, const VectorValues& delta) const;

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
  boost::tuple<double, int> computeStepSize(
      const InequalityFactorGraph& workingSet, const VectorValues& xk,
      const VectorValues& p) const;

  /// Iterate 1 step, return a new state with a new workingSet and values
  QPState iterate(const QPState& state) const;

  /// Identify active constraints based on initial values.
  InequalityFactorGraph identifyActiveConstraints(
      const InequalityFactorGraph& inequalities,
      const VectorValues& initialValues, const VectorValues& duals =
          VectorValues(), bool useWarmStart = true) const;

  /**
   * Optimize with provided initial values
   * For this version, it is the responsibility of the caller to provide
   * a feasible initial value, otherwise, an exception will be thrown.
   * @return a pair of <primal, dual> solutions
   */
  std::pair<VectorValues, VectorValues> optimize(
      const VectorValues& initialValues, const VectorValues& duals =
          VectorValues(), bool useWarmStart = true) const;

};

} // namespace gtsam
