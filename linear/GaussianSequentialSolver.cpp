/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SequentialSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 19, 2010
 */

#include <gtsam/linear/GaussianSequentialSolver.h>

#include <gtsam/inference/GenericSequentialSolver-inl.h>

namespace gtsam {

/* ************************************************************************* */
GaussianSequentialSolver::GaussianSequentialSolver(const FactorGraph<GaussianFactor>& factorGraph) :
    Base(factorGraph) {}

/* ************************************************************************* */
GaussianSequentialSolver::shared_ptr GaussianSequentialSolver::Create(const FactorGraph<GaussianFactor>& factorGraph) {
  return shared_ptr(new GaussianSequentialSolver(factorGraph));
}

/* ************************************************************************* */
GaussianSequentialSolver::shared_ptr GaussianSequentialSolver::update(const FactorGraph<GaussianFactor>& factorGraph) const {
  // We do not yet have code written to update the elimination tree, so we just
  // create a new solver.
  return Create(factorGraph);
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr GaussianSequentialSolver::eliminate() const {
  return Base::eliminate();
}

/* ************************************************************************* */
VectorValues::shared_ptr GaussianSequentialSolver::optimize() const {

  static const bool debug = false;

  if(debug) this->factors_.print("GaussianSequentialSolver, eliminating ");
  if(debug) this->eliminationTree_->print("GaussianSequentialSolver, elimination tree ");

  // Eliminate using the elimination tree
  GaussianBayesNet::shared_ptr bayesNet(this->eliminate());

  if(debug) bayesNet->print("GaussianSequentialSolver, Bayes net ");

  // Allocate the solution vector if it is not already allocated
//  VectorValues::shared_ptr solution = allocateVectorValues(*bayesNet);

  // Back-substitute
  VectorValues::shared_ptr solution(gtsam::optimize_(*bayesNet));

  if(debug) solution->print("GaussianSequentialSolver, solution ");

  return solution;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianSequentialSolver::marginal(Index j) const {
  return Base::marginal(j);
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr GaussianSequentialSolver::joint(const std::vector<Index>& js) const {
  return GaussianFactorGraph::shared_ptr(new GaussianFactorGraph(*Base::joint(js)));
}

}
