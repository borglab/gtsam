/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     LPInitSolver.h
 * @brief    This LPInitSolver implements the strategy in Matlab.
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/linear/LP.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {
/**
 * This LPInitSolver implements the strategy in Matlab:
 * http://www.mathworks.com/help/optim/ug/linear-programming-algorithms.html#brozyzb-9
 * Solve for x and y:
 *    min y
 *    st Ax = b
 *       Cx - y <= d
 * where \f$y \in R\f$, \f$x \in R^n\f$, and Ax = b and Cx <= d is the constraints of the original problem.
 *
 * If the solution for this problem {x*,y*} has y* <= 0, we'll have x* a feasible initial point
 * of the original problem
 * otherwise, if y* > 0 or the problem has no solution, the original problem is infeasible.
 *
 * The initial value of this initial problem can be found by solving
 *    min   ||x||^2
 *    s.t.   Ax = b
 * to have a solution x0
 * then y = max_j ( Cj*x0  - dj )  -- due to the constraints y >= Cx - d
 *
 * WARNING: If some xj in the inequality constraints does not exist in the equality constraints,
 * set them as zero for now. If that is the case, the original problem doesn't have a unique
 * solution (it could be either infeasible or unbounded).
 * So, if the initialization fails because we enforce xj=0 in the problematic
 * inequality constraint, we can't conclude that the problem is infeasible.
 * However, whether it is infeasible or unbounded, we don't have a unique solution anyway.
 */
class GTSAM_UNSTABLE_EXPORT LPInitSolver {
private:
  const LP& lp_;

public:
  /// Construct with an LP problem
  LPInitSolver(const LP& lp) : lp_(lp) {}

  ///@return a feasible initialization point
  VectorValues solve() const;

private:
  /// build initial LP
  LP::shared_ptr buildInitialLP(Key yKey) const;

  /**
   * Build the following graph to solve for an initial value of the initial problem
   *    min   ||x||^2    s.t.   Ax = b
   */
  GaussianFactorGraph::shared_ptr buildInitOfInitGraph() const;

  /// y = max_j ( Cj*x0  - dj )  -- due to the inequality constraints y >= Cx - d
  double compute_y0(const VectorValues& x0) const;

  /// Collect all terms of a factor into a container.
  std::vector<std::pair<Key, Matrix>> collectTerms(
      const LinearInequality& factor) const;

  /// Turn Cx <= d into Cx - y <= d factors
  InequalityFactorGraph addSlackVariableToInequalities(Key yKey,
      const InequalityFactorGraph& inequalities) const;

  // friend class for unit-testing private methods
  friend class LPInitSolverInitializationTest;
};

}
