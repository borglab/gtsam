/**
 * @file    GenericSequentialSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/GenericSequentialSolver.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/EliminationTree-inl.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/inference-inl.h>

#include <boost/foreach.hpp>

namespace gtsam {

/* ************************************************************************* */
template<class FACTOR>
GenericSequentialSolver<FACTOR>::GenericSequentialSolver(const FactorGraph<FACTOR>& factorGraph) :
    structure_(factorGraph),
    eliminationTree_(EliminationTree<FACTOR>::Create(factorGraph, structure_)) {
  factors_.push_back(factorGraph);
}

/* ************************************************************************* */
template<class FACTOR>
typename BayesNet<typename FACTOR::Conditional>::shared_ptr GenericSequentialSolver<FACTOR>::eliminate() const {
  return eliminationTree_->eliminate();
}

/* ************************************************************************* */
template<class FACTOR>
typename FactorGraph<FACTOR>::shared_ptr GenericSequentialSolver<FACTOR>::joint(const std::vector<Index>& js) const {

  // Compute a COLAMD permutation with the marginal variable constrained to the end.
  Permutation::shared_ptr permutation(Inference::PermutationCOLAMD(structure_, js));
  Permutation::shared_ptr permutationInverse(permutation->inverse());

  // Permute the factors - NOTE that this permutes the original factors, not
  // copies.  Other parts of the code may hold shared_ptr's to these factors so
  // we must undo the permutation before returning.
  BOOST_FOREACH(const typename FACTOR::shared_ptr& factor, factors_) {
    if(factor)
      factor->permuteWithInverse(*permutationInverse);
  }

  // Eliminate all variables
  typename BayesNet<typename FACTOR::Conditional>::shared_ptr bayesNet(
      EliminationTree<FACTOR>::Create(factors_)->eliminate());

  // Undo the permuation on the original factors and on the structure.
  BOOST_FOREACH(const typename FACTOR::shared_ptr& factor, factors_) {
    if(factor)
      factor->permuteWithInverse(*permutation);
  }

  // Take the joint marginal from the Bayes net.
  typename FactorGraph<FACTOR>::shared_ptr joint(new FactorGraph<FACTOR>);
  joint->reserve(js.size());
  typename BayesNet<typename FACTOR::Conditional>::const_reverse_iterator conditional = bayesNet->rbegin();
  for(size_t i = 0; i < js.size(); ++i) {
    joint->push_back(typename FACTOR::shared_ptr(new FACTOR(**(conditional++)))); }

  // Undo the permutation on the eliminated joint marginal factors
  BOOST_FOREACH(const typename FACTOR::shared_ptr& factor, *joint) {
    factor->permuteWithInverse(*permutation); }

  return joint;
}

/* ************************************************************************* */
template<class FACTOR>
typename FACTOR::shared_ptr GenericSequentialSolver<FACTOR>::marginal(Index j) const {
  // Create a container for the one variable index
  vector<Index> js(1); js[0] = j;

  // Call joint and return the only factor in the factor graph it returns
  return (*this->joint(js))[0];
}

}
