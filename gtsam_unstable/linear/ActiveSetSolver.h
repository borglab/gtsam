/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     ActiveSetSolver.h
 * @brief    Active set method for solving LP, QP problems
 * @author   Ivan Dario Jimenez
 * @author   Duy Nguyen Ta
 * @date     1/25/16
 */
#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_unstable/linear/InequalityFactorGraph.h>
#include <boost/range/adaptor/map.hpp>

namespace gtsam {

/**
 * This class implements the active set algorithm for solving convex
 * Programming problems.
 *
 * @tparam PROBLEM Type of the problem to solve, e.g. LP (linear program) or
 *                 QP (quadratic program).
 * @tparam POLICY specific detail policy tailored for the particular program
 * @tparam INITSOLVER Solver for an initial feasible solution of this problem.
 */
template <class PROBLEM, class POLICY, class INITSOLVER>
class ActiveSetSolver {
public:
  /// This struct contains the state information for a single iteration
  struct State {
    VectorValues values;  //!< current best values at each step
    VectorValues duals;   //!< current values of dual variables at each step
    InequalityFactorGraph workingSet; /*!< keep track of current active/inactive
                                           inequality constraints */
    bool converged;     //!< True if the algorithm has converged to a solution
    size_t iterations;  /*!< Number of iterations. Incremented at the end of
                        each iteration. */

    /// Default constructor
    State()
        : values(), duals(), workingSet(), converged(false), iterations(0) {}

    /// Constructor with initial values
    State(const VectorValues& initialValues, const VectorValues& initialDuals,
          const InequalityFactorGraph& initialWorkingSet, bool _converged,
          size_t _iterations)
        : values(initialValues),
          duals(initialDuals),
          workingSet(initialWorkingSet),
          converged(_converged),
          iterations(_iterations) {}
  };

protected:
  const PROBLEM& problem_;  //!< the particular [convex] problem to solve
  VariableIndex equalityVariableIndex_,
      inequalityVariableIndex_;  /*!< index to corresponding factors to build
                                 dual graphs */
  KeySet constrainedKeys_;  /*!< all constrained keys, will become factors in
                                 dual graphs */

  /// Vector of key matrix pairs. Matrices are usually the A term for a factor.
  typedef std::vector<std::pair<Key, Matrix> > TermsContainer;

public:
  /// Constructor
  ActiveSetSolver(const PROBLEM& problem) :  problem_(problem) {
    equalityVariableIndex_ = VariableIndex(problem_.equalities);
    inequalityVariableIndex_ = VariableIndex(problem_.inequalities);
    constrainedKeys_ = problem_.equalities.keys();
    constrainedKeys_.merge(problem_.inequalities.keys());
  }

  /**
   * Optimize with provided initial values
   * For this version, it is the responsibility of the caller to provide
   * a feasible initial value, otherwise, an exception will be thrown.
   * @return a pair of <primal, dual> solutions
   */
  std::pair<VectorValues, VectorValues> optimize(
      const VectorValues& initialValues,
      const VectorValues& duals = VectorValues(),
      bool useWarmStart = false) const;

  /**
   * For this version the caller will not have to provide an initial value
   * @return a pair of <primal, dual> solutions
   */
  std::pair<VectorValues, VectorValues> optimize() const;

protected:
  /**
   * Compute minimum step size alpha to move from the current point @p xk to the
   * next feasible point along a direction @p p:  x' = xk + alpha*p,
   * where alpha \f$\in\f$ [0,maxAlpha].
   *
   * For QP, maxAlpha = 1. For LP: maxAlpha = Inf.
   *
   * @return a tuple of (minAlpha, closestFactorIndex) where closestFactorIndex
   * is the closest inactive inequality constraint that blocks xk to move
   * further and that has the minimum alpha, or (-1, maxAlpha) if there is no
   * such inactive blocking constraint.
   *
   * If there is a blocking constraint, the closest one will be added to the
   * working set and become active in the next iteration.
   */
  boost::tuple<double, int> computeStepSize(
      const InequalityFactorGraph& workingSet, const VectorValues& xk,
      const VectorValues& p, const double& maxAlpha) const;

  /**
   * Finds the active constraints in the given factor graph and returns the
   * Dual Jacobians used to build a dual factor graph.
   */
  template<typename FACTOR>
  TermsContainer collectDualJacobians(Key key, const FactorGraph<FACTOR>& graph,
      const VariableIndex& variableIndex) const {
    /*
     * Iterates through each factor in the factor graph and checks
     * whether it's active. If the factor is active it reutrns the A
     * term of the factor.
     */
    TermsContainer Aterms;
    if (variableIndex.find(key) != variableIndex.end()) {
      for (size_t factorIx : variableIndex[key]) {
        typename FACTOR::shared_ptr factor = graph.at(factorIx);
        if (!factor->active())
          continue;
        Matrix Ai = factor->getA(factor->find(key)).transpose();
        Aterms.push_back(std::make_pair(factor->dualKey(), Ai));
      }
    }
    return Aterms;
  }

  /**
   * Creates a dual factor from the current workingSet and the key of the
   * the variable used to created the dual factor.
   */
  JacobianFactor::shared_ptr createDualFactor(
    Key key, const InequalityFactorGraph& workingSet,
    const VectorValues& delta) const;

public: /// Just for testing...

  /// Builds a dual graph from the current working set.
  GaussianFactorGraph buildDualGraph(const InequalityFactorGraph &workingSet,
                                     const VectorValues &delta) const;

  /**
   * Build a working graph of cost, equality and active inequality constraints
   * to solve at each iteration.
   * @param  workingSet the collection of all cost and constrained factors
   * @param  xk   current solution, used to build a special quadratic cost in LP
   * @return      a new better solution
   */
  GaussianFactorGraph buildWorkingGraph(
      const InequalityFactorGraph& workingSet,
      const VectorValues& xk = VectorValues()) const;

  /// Iterate 1 step, return a new state with a new workingSet and values
  State iterate(const State& state) const;

  /// Identify active constraints based on initial values.
  InequalityFactorGraph identifyActiveConstraints(
      const InequalityFactorGraph& inequalities,
      const VectorValues& initialValues,
      const VectorValues& duals = VectorValues(),
      bool useWarmStart = false) const;

  /// Identifies active constraints that shouldn't be active anymore.
  int identifyLeavingConstraint(const InequalityFactorGraph& workingSet,
      const VectorValues& lambdas) const;

};

/**
 * Find the max key in a problem.
 * Useful to determine unique keys for additional slack variables
 */
template <class PROBLEM>
Key maxKey(const PROBLEM& problem) {
  auto keys = problem.cost.keys();
  Key maxKey = *std::max_element(keys.begin(), keys.end());
  if (!problem.equalities.empty())
    maxKey = std::max(maxKey, *problem.equalities.keys().rbegin());
  if (!problem.inequalities.empty())
    maxKey = std::max(maxKey, *problem.inequalities.keys().rbegin());
  return maxKey;
}

} // namespace gtsam

#include <gtsam_unstable/linear/ActiveSetSolver-inl.h>