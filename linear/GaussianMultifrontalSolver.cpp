/**
 * @file    GaussianMultifrontalSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#include <gtsam/linear/GaussianMultifrontalSolver.h>

#include <gtsam/inference/GenericMultifrontalSolver-inl.h>

namespace gtsam {

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>& factorGraph) :
    junctionTree_(new GaussianJunctionTree(factorGraph)) {}

/* ************************************************************************* */
BayesTree<GaussianConditional>::sharedClique GaussianMultifrontalSolver::eliminate() const {
  return junctionTree_->eliminate();
}

/* ************************************************************************* */
VectorValues::shared_ptr GaussianMultifrontalSolver::optimize() const {
  return VectorValues::shared_ptr(new VectorValues(junctionTree_->optimize()));
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianMultifrontalSolver::marginal(Index j) const {
  BayesTree<GaussianConditional> bayesTree;
  bayesTree.insert(junctionTree_->eliminate());
  return bayesTree.marginal(j);
}

}
