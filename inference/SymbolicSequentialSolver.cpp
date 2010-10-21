/**
 * @file    SymbolicSequentialSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#include <gtsam/inference/SymbolicSequentialSolver.h>
#include <gtsam/inference/GenericSequentialSolver-inl.h>

namespace gtsam {


/* ************************************************************************* */
SymbolicSequentialSolver::SymbolicSequentialSolver(const FactorGraph<IndexFactor>& factorGraph) :
    Base(factorGraph) {}

/* ************************************************************************* */
typename BayesNet<IndexConditional>::shared_ptr SymbolicSequentialSolver::eliminate() const {
  return Base::eliminate();
}

/* ************************************************************************* */
SymbolicFactorGraph::shared_ptr SymbolicSequentialSolver::joint(const std::vector<Index>& js) const {
  return SymbolicFactorGraph::shared_ptr(new SymbolicFactorGraph(*Base::joint(js)));
}

}
