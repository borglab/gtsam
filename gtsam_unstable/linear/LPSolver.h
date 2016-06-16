/**
 * @file     LPSolver.h
 * @brief    Class used to solve Linear Programming Problems as defined in LP.h
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#pragma once

#include <gtsam_unstable/linear/LPState.h>
#include <gtsam_unstable/linear/LP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/LinearCost.h>
#include <gtsam/linear/VectorValues.h>

#include <boost/range/adaptor/map.hpp>

namespace gtsam {


class LPSolver: public ActiveSetSolver {
  const LP &lp_; //!< the linear programming problem
public:
  /// Constructor
  LPSolver(const LP &lp);

  const LP &lp() const {
    return lp_;
  }

  /*
   * This function performs an iteration of the Active Set Method for solving
   * LP problems. At the end of this iteration the problem should either be found
   * to be unfeasible, solved or the current state changed to reflect a new
   * working set.
   */
  LPState iterate(const LPState &state) const;

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
  GaussianFactorGraph buildCostFunction(const VectorValues &xk) const;

  GaussianFactorGraph buildWorkingGraph(
    const InequalityFactorGraph& workingSet, const VectorValues& xk) const;

  /*
   * A dual factor takes the objective function and a set of constraints.
   * It then creates a least-square approximation of the lagrangian multipliers
   * for the following problem: f' = - lambda * g' where f is the objection
   * function g are dual factors and lambda is the lagrangian multiplier.
   */
  JacobianFactor::shared_ptr createDualFactor(Key key,
      const InequalityFactorGraph &workingSet, const VectorValues &delta) const;

  /// TODO(comment)
  boost::tuple<double, int> computeStepSize(
      const InequalityFactorGraph &workingSet, const VectorValues &xk,
      const VectorValues &p) const;

  /*
   * Given an initial value this function determine which constraints are active
   * which can be used to initialize the working set.
   * A constraint Ax <= b  is active if we have an x' s.t. Ax' = b
   */
  InequalityFactorGraph identifyActiveConstraints(
      const InequalityFactorGraph &inequalities,
      const VectorValues &initialValues, const VectorValues &duals,
      bool useWarmStart = false) const;

  /** Optimize with the provided feasible initial values
   * TODO: throw exception if the initial values is not feasible wrt inequality constraints
   * TODO: comment duals
   */
  pair<VectorValues, VectorValues> optimize(const VectorValues &initialValues,
      const VectorValues &duals = VectorValues(), bool useWarmStart = false) const;

  /**
   * Optimize without initial values.
   */
  pair<VectorValues, VectorValues> optimize() const;
};
} // namespace gtsam
