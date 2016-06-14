/**
 * @file     ActiveSetSolver.h
 * @brief    Abstract class above for solving problems with the abstract set method.
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
 * This is a base class for all implementations of the active set algorithm for solving 
 * Programming problems. It provides services and variables all active set implementations
 * share.
 */
class ActiveSetSolver {
protected:
  KeySet constrainedKeys_; //!< all constrained keys, will become factors in dual graphs
  GaussianFactorGraph baseGraph_; //!< factor graphs of cost factors and linear equalities.
  //!< used to initialize the working set factor graph,
  //!< to which active inequalities will be added
  VariableIndex costVariableIndex_, equalityVariableIndex_,
      inequalityVariableIndex_; //!< index to corresponding factors to build dual graphs

public:
  typedef std::vector<std::pair<Key, Matrix> > TermsContainer; //!< vector of key matrix pairs
  //Matrices are usually the A term for a factor.
  /**
   * Creates a dual factor from the current workingSet and the key of the
   * the variable used to created the dual factor.
   */
  virtual JacobianFactor::shared_ptr createDualFactor(Key key,
      const InequalityFactorGraph& workingSet,
      const VectorValues& delta) const = 0;

  /**
   * Finds the active constraints in the given factor graph and returns the 
   * Dual Jacobians used to build a dual factor graph.
   */
  template<typename FACTOR>
  TermsContainer collectDualJacobians(Key key,
      const FactorGraph<FACTOR>& graph,
      const VariableIndex& variableIndex) const {
    /*
     * Iterates through each factor in the factor graph and checks 
     * whether it's active. If the factor is active it reutrns the A 
     * term of the factor.
     */
    TermsContainer Aterms;
    if (variableIndex.find(key) != variableIndex.end()) {
    for(size_t factorIx: variableIndex[key]) {
      typename FACTOR::shared_ptr factor = graph.at(factorIx);
      if (!factor->active()) continue;
      Matrix Ai = factor->getA(factor->find(key)).transpose();
      Aterms.push_back(std::make_pair(factor->dualKey(), Ai));
    }
  }
  return Aterms;
}
/**
 * Identifies active constraints that shouldn't be active anymore.
 */
int identifyLeavingConstraint(const InequalityFactorGraph& workingSet,
    const VectorValues& lambdas) const;

/**
 * Builds a dual graph from the current working set.
 */
GaussianFactorGraph::shared_ptr buildDualGraph(
    const InequalityFactorGraph& workingSet, const VectorValues& delta) const;

protected:
/**
 * Protected constructor because this class doesn't have any meaning without
 * a concrete Programming problem to solve.
 */
ActiveSetSolver() :
    constrainedKeys_() {
}

/**
 * Computes the distance to move from the current point being examined to the next 
 * location to be examined by the graph. This should only be used where there are less 
 * than two constraints active.
 */
boost::tuple<double, int> computeStepSize(
    const InequalityFactorGraph& workingSet, const VectorValues& xk,
    const VectorValues& p, const double& startAlpha) const;
};
} // namespace gtsam
