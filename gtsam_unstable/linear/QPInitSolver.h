/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPInitSolver.h
 * @brief    This finds a feasible solution for a QP problem
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     6/16/16
 */

#pragma once

#include <gtsam/config.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam_unstable/linear/LPInitSolver.h>
#include <gtsam_unstable/linear/QP.h>

#include <algorithm>

namespace gtsam {
namespace internal {

/* ************************************************************************* */
inline Key maxKey(const QP& qp) {
  Key maxKey = 0;
  bool initialized = false;
  auto update = [&](const KeySet& keys) {
    if (!keys.empty()) {
      maxKey = initialized ? std::max(maxKey, *keys.rbegin()) : *keys.rbegin();
      initialized = true;
    }
  };
  update(qp.cost.keys());
  update(qp.equalities.keys());
  update(qp.inequalities.keys());
  return maxKey;
}

}  // namespace internal

/**
 * This class finds a feasible solution for a QP problem.
 * This uses the Matlab strategy for initialization
 * For details, see
 * http://www.mathworks.com/help/optim/ug/quadratic-programming-algorithms.html#brrzwpf-22
 *
 * @deprecated QPSolver now uses constrained-module phase-I initialization.
 */
class QPInitSolver {
  const QP& qp_;

 public:
  /// Constructor with a QP problem
  QPInitSolver(const QP& qp) : qp_(qp) {}

  ///@return a feasible initialization point
  VectorValues solve() const {
    // Make an LP with any linear cost function. It doesn't matter for
    // initialization.
    LP initProblem;
    // make an unrelated key for a random variable cost
    Key newKey = internal::maxKey(qp_) + 1;
    initProblem.cost = LinearCost(newKey, Vector::Ones(1));
    initProblem.equalities = qp_.equalities;
    initProblem.inequalities = qp_.inequalities;
    LPInitSolver initSolver(initProblem);
    return initSolver.solve();
  }
};

}  // namespace gtsam

#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43
