/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     LPSolver.h
 * @brief    Policy of ActiveSetSolver to solve Linear Programming Problems
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     6/16/16
 */

#include <gtsam_unstable/linear/LP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/LPInitSolver.h>

#include <limits>
#include <algorithm>

namespace gtsam {

/// Policy for ActivetSetSolver to solve Linear Programming \sa LP problems
struct LPPolicy {
  /// Maximum alpha for line search x'=xk + alpha*p, where p is the cost gradient
  /// For LP, maxAlpha = Infinity
  static constexpr double maxAlpha = std::numeric_limits<double>::infinity();

  /**
   * Create the factor ||x-xk - (-g)||^2 where xk is the current feasible solution
   * on the constraint surface and g is the gradient of the linear cost,
   * i.e. -g is the direction we wish to follow to decrease the cost.
   *
   * Essentially, we try to match the direction d = x-xk with -g as much as possible
   * subject to the condition that x needs to be on the constraint surface, i.e., d is
   * along the surface's subspace.
   *
   * The least-square solution of this quadratic subject to a set of linear constraints
   * is the projection of the gradient onto the constraints' subspace
   */
  static GaussianFactorGraph buildCostFunction(const LP& lp,
                                               const VectorValues& xk) {
    GaussianFactorGraph graph;
    for (LinearCost::const_iterator it = lp.cost.begin(); it != lp.cost.end();
         ++it) {
      size_t dim = lp.cost.getDim(it);
      Vector b = xk.at(*it) - lp.cost.getA(it).transpose();  // b = xk-g
      graph.emplace_shared<JacobianFactor>(*it, Matrix::Identity(dim, dim), b);
    }

    KeySet allKeys = lp.inequalities.keys();
    allKeys.merge(lp.equalities.keys());
    allKeys.merge(KeySet(lp.cost.keys()));
    // Add corresponding factors for all variables that are not explicitly in
    // the cost function. Gradients of the cost function wrt to these variables
    // are zero (g=0), so b=xk
    if (lp.cost.keys().size() != allKeys.size()) {
      KeySet difference;
      std::set_difference(allKeys.begin(), allKeys.end(), lp.cost.begin(),
                          lp.cost.end(),
                          std::inserter(difference, difference.end()));
      for (Key k : difference) {
        size_t dim = lp.constrainedKeyDimMap().at(k);
        graph.emplace_shared<JacobianFactor>(k, Matrix::Identity(dim, dim), xk.at(k));
      }
    }
    return graph;
  }
};

using LPSolver = ActiveSetSolver<LP, LPPolicy, LPInitSolver>;

}
