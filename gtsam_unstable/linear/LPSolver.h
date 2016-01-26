/**
 * @file     LPSolver.h
 * @brief    Class used to solve Linear Programming Problems as defined in LP.h
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#pragma once

#include <gtsam_unstable/linear/LPState.h>
#include <gtsam_unstable/linear/LP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <boost/range/adaptor/map.hpp>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

typedef std::map<Key, size_t> KeyDimMap;

class LPSolver: public ActiveSetSolver {
  const LP& lp_; //!< the linear programming problem
  KeyDimMap keysDim_; //!< key-dim map of all variables in the constraints, used to create zero priors

public:
  /// Constructor
  LPSolver(const LP& lp);

  const LP& lp() const {
    return lp_;
  }
  const KeyDimMap& keysDim() const {
    return keysDim_;
  }

  //******************************************************************************
  template<class LinearGraph>
  KeyDimMap collectKeysDim(const LinearGraph& linearGraph) const {
    KeyDimMap keysDim;
    BOOST_FOREACH(const typename LinearGraph::sharedFactor& factor, linearGraph) {
      if (!factor) continue;
      BOOST_FOREACH(Key key, factor->keys())
      keysDim[key] = factor->getDim(factor->find(key));
    }
    return keysDim;
  }

  //******************************************************************************
  /**
   * Create a zero prior for any keys in the graph that don't exist in the cost
   */
  GaussianFactorGraph::shared_ptr createZeroPriors(const KeyVector& costKeys,
      const KeyDimMap& keysDim) const;

  //******************************************************************************
  LPState iterate(const LPState& state) const;

  //******************************************************************************
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
      const LinearCost& cost, const VectorValues& xk) const;

  /// Find solution with the current working set
  VectorValues solveWithCurrentWorkingSet(const VectorValues& xk,
      const InequalityFactorGraph& workingSet) const;

//******************************************************************************
  JacobianFactor::shared_ptr createDualFactor(Key key,
      const InequalityFactorGraph& workingSet, const VectorValues& delta) const;

//******************************************************************************
  boost::tuple<double, int> computeStepSize(
      const InequalityFactorGraph& workingSet, const VectorValues& xk,
      const VectorValues& p) const;

//******************************************************************************
  InequalityFactorGraph identifyActiveConstraints(
      const InequalityFactorGraph& inequalities,
      const VectorValues& initialValues, const VectorValues& duals) const;

//******************************************************************************
  /** Optimize with the provided feasible initial values
   * TODO: throw exception if the initial values is not feasible wrt inequality constraints
   */
  pair<VectorValues, VectorValues> optimize(const VectorValues& initialValues,
      const VectorValues& duals = VectorValues()) const;

//******************************************************************************
  /**
   * Optimize without initial values
   * TODO: Find a feasible initial solution wrt inequality constraints
   */
//  pair<VectorValues, VectorValues> optimize() const {
//
//    // Initialize workingSet from the feasible initialValues
//    InequalityFactorGraph workingSet = identifyActiveConstraints(
//        lp_.inequalities, initialValues, duals);
//    LPState state(initialValues, duals, workingSet, false, 0);
//
//    /// main loop of the solver
//    while (!state.converged) {
//      state = iterate(state);
//    }
//
//    return make_pair(state.values, state.duals);
//  }
};
}
