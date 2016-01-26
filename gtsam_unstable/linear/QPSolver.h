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
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/linear/VectorValues.h>
#include <gtsam_unstable/linear/QP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/QPState.h>

#include <vector>
#include <set>

namespace gtsam {
/**
 * This QPSolver uses the active set method to solve a quadratic programming problem
 * defined in the QP struct.
 * Note: This version of QPSolver only works with a feasible initial value.
 */
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
  /// @}

  boost::tuple<double, int> computeStepSize(
      const InequalityFactorGraph& workingSet, const VectorValues& xk,
      const VectorValues& p) const;

  /** Iterate 1 step, return a new state with a new workingSet and values */
  QPState iterate(const QPState& state) const;

  /**
   * Identify active constraints based on initial values.
   */
  InequalityFactorGraph identifyActiveConstraints(
      const InequalityFactorGraph& inequalities,
      const VectorValues& initialValues, const VectorValues& duals =
          VectorValues(), bool useWarmStart = true) const;

  /** Optimize with a provided initial values
   * For this version, it is the responsibility of the caller to provide
   * a feasible initial value, otherwise, an exception will be thrown.
   * @return a pair of <primal, dual> solutions
   */
  std::pair<VectorValues, VectorValues> optimize(
      const VectorValues& initialValues, const VectorValues& duals =
          VectorValues(), bool useWarmStart = true) const;

};

} /* namespace gtsam */
