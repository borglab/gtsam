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

typedef std::map<Key, size_t> KeyDimMap;

class LPSolver: public ActiveSetSolver {
  const LP &lp_; //!< the linear programming problem
  KeyDimMap keysDim_; //!< key-dim map of all variables in the constraints, used to create zero priors
  std::vector<size_t> addedZeroPriorsIndex_;
public:
  /// Constructor
  LPSolver(const LP &lp);

  const LP &lp() const {
    return lp_;
  }

  const KeyDimMap &keysDim() const {
    return keysDim_;
  }

  /*
   * Iterates through every factor in a linear graph and generates a
   * mapping between every factor key and it's corresponding dimensionality.
   */
  template<class LinearGraph>
  KeyDimMap collectKeysDim(const LinearGraph &linearGraph) const {
    KeyDimMap keysDim;
    for (const typename LinearGraph::sharedFactor &factor : linearGraph) {
      if (!factor)
        continue;
      for (Key key : factor->keys())
        keysDim[key] = factor->getDim(factor->find(key));
    }
    return keysDim;
  }

  /// Create a zero prior for any keys in the graph that don't exist in the cost
  GaussianFactorGraph::shared_ptr createZeroPriors(const KeyVector &costKeys,
      const KeyDimMap &keysDim) const;

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
  GaussianFactorGraph::shared_ptr createLeastSquareFactors(
      const LinearCost &cost, const VectorValues &xk) const;

  /// Find solution with the current working set
  VectorValues solveWithCurrentWorkingSet(const VectorValues &xk,
      const InequalityFactorGraph &workingSet) const;

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
      const VectorValues &initialValues, const VectorValues &duals) const;

  /** Optimize with the provided feasible initial values
   * TODO: throw exception if the initial values is not feasible wrt inequality constraints
   * TODO: comment duals
   */
  pair<VectorValues, VectorValues> optimize(const VectorValues &initialValues,
      const VectorValues &duals = VectorValues()) const;

  /**
   * Optimize without initial values.
   */
  pair<VectorValues, VectorValues> optimize() const;
};
} // namespace gtsam
