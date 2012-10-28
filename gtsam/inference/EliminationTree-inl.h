/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    EliminationTree-inl.h
 * @author  Frank Dellaert
 * @date    Oct 13, 2010
 */
#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/IndexConditional.h>

#include <boost/foreach.hpp>
#include <boost/static_assert.hpp>

#include <iostream>
#include <set>
#include <vector>

namespace gtsam {

/* ************************************************************************* */
template<class FACTOR>
typename EliminationTree<FACTOR>::sharedFactor EliminationTree<FACTOR>::eliminate_(
  Eliminate function, Conditionals& conditionals) const {

    static const bool debug = false;

    if(debug) std::cout << "ETree: eliminating " << this->key_ << std::endl;

    if(this->key_ < conditionals.size()) { // If it is requested to eliminate the current variable
      // Create the list of factors to be eliminated, initially empty, and reserve space
      FactorGraph<FACTOR> factors;
      factors.reserve(this->factors_.size() + this->subTrees_.size());

      // Add all factors associated with the current node
      factors.push_back(this->factors_.begin(), this->factors_.end());

      // for all subtrees, eliminate into Bayes net and a separator factor, added to [factors]
      BOOST_FOREACH(const shared_ptr& child, subTrees_)
        factors.push_back(child->eliminate_(function, conditionals)); // TODO: spawn thread
      // TODO: wait for completion of all threads

      // Combine all factors (from this node and from subtrees) into a joint factor
      typename FactorGraph<FACTOR>::EliminationResult
        eliminated(function(factors, 1));
      conditionals[this->key_] = eliminated.first;

      if(debug) std::cout << "Eliminated " << this->key_ << " to get:\n";
      if(debug) eliminated.first->print("Conditional: ");
      if(debug) eliminated.second->print("Factor: ");

      return eliminated.second;
    } else {
      // Eliminate each child but discard the result.
      BOOST_FOREACH(const shared_ptr& child, subTrees_) {
        (void)child->eliminate_(function, conditionals);
      }
      return sharedFactor(); // Return a NULL factor
    }
}

/* ************************************************************************* */
template<class FACTOR>
std::vector<Index> EliminationTree<FACTOR>::ComputeParents(const VariableIndex& structure) {

  // Number of factors and variables
  const size_t m = structure.nFactors();
  const size_t n = structure.size();

  static const Index none = std::numeric_limits<Index>::max();

  // Allocate result parent vector and vector of last factor columns
  std::vector<Index> parents(n, none);
  std::vector<Index> prevCol(m, none);

  // for column j \in 1 to n do
  for (Index j = 0; j < n; j++) {
    // for row i \in Struct[A*j] do
    BOOST_FOREACH(const size_t i, structure[j]) {
      if (prevCol[i] != none) {
        Index k = prevCol[i];
        // find root r of the current tree that contains k
        Index r = k;
        while (parents[r] != none)
          r = parents[r];
        if (r != j) parents[r] = j;
      }
      prevCol[i] = j;
    }
  }

  return parents;
}

/* ************************************************************************* */
template<class FACTOR>
template<class DERIVEDFACTOR>
typename EliminationTree<FACTOR>::shared_ptr EliminationTree<FACTOR>::Create(
    const FactorGraph<DERIVEDFACTOR>& factorGraph,
    const VariableIndex& structure) {

  static const bool debug = false;
  gttic(ET_Create1);

  gttic(ET_ComputeParents);
  // Compute the tree structure
  std::vector<Index> parents(ComputeParents(structure));
  gttoc(ET_ComputeParents);

  // Number of variables
  const size_t n = structure.size();

  static const Index none = std::numeric_limits<Index>::max();

  // Create tree structure
  gttic(assemble_tree);
  std::vector<shared_ptr> trees(n);
  for (Index k = 1; k <= n; k++) {
    Index j = n - k;  // Start at the last variable and loop down to 0
    trees[j].reset(new EliminationTree(j));  // Create a new node on this variable
    if (parents[j] != none)  // If this node has a parent, add it to the parent's children
      trees[parents[j]]->add(trees[j]);
    else if(!structure[j].empty() && j != n - 1) // If a node other than the last has no parents, this is a forest
      throw DisconnectedGraphException();
  }
  gttoc(assemble_tree);

  // Hang factors in right places
  gttic(hang_factors);
  BOOST_FOREACH(const typename boost::shared_ptr<DERIVEDFACTOR>& factor, factorGraph) {
    if(factor && factor->size() > 0) {
      Index j = *std::min_element(factor->begin(), factor->end());
      if(j < structure.size())
        trees[j]->add(factor);
    }
  }
  gttoc(hang_factors);

  if(debug)
    trees.back()->print("ETree: ");

  // Check that this is not null
  assert(trees.back().get());
  return trees.back();
}

/* ************************************************************************* */
template<class FACTOR>
template<class DERIVEDFACTOR>
typename EliminationTree<FACTOR>::shared_ptr
EliminationTree<FACTOR>::Create(const FactorGraph<DERIVEDFACTOR>& factorGraph) {

  gttic(ET_Create2);
  // Build variable index
  const VariableIndex variableIndex(factorGraph);

  // Build elimination tree
  return Create(factorGraph, variableIndex);
}

/* ************************************************************************* */
template<class FACTORGRAPH>
void EliminationTree<FACTORGRAPH>::print(const std::string& name,
    const IndexFormatter& formatter) const {
  std::cout << name << " (" << formatter(key_) << ")" << std::endl;
  BOOST_FOREACH(const sharedFactor& factor, factors_) {
    factor->print(name + "  ", formatter); }
  BOOST_FOREACH(const shared_ptr& child, subTrees_) {
    child->print(name + "  ", formatter); }
}

/* ************************************************************************* */
template<class FACTORGRAPH>
bool EliminationTree<FACTORGRAPH>::equals(const EliminationTree<FACTORGRAPH>& expected, double tol) const {
  if(this->key_ == expected.key_ && this->factors_ == expected.factors_
      && this->subTrees_.size() == expected.subTrees_.size()) {
    typename SubTrees::const_iterator this_subtree = this->subTrees_.begin();
    typename SubTrees::const_iterator expected_subtree = expected.subTrees_.begin();
    while(this_subtree != this->subTrees_.end())
      if( ! (*(this_subtree++))->equals(**(expected_subtree++), tol))
        return false;
    return true;
  } else
    return false;
}

/* ************************************************************************* */
template<class FACTOR>
typename EliminationTree<FACTOR>::BayesNet::shared_ptr
  EliminationTree<FACTOR>::eliminatePartial(typename EliminationTree<FACTOR>::Eliminate function, size_t nrToEliminate) const {

  // call recursive routine
  gttic(ET_recursive_eliminate);
  if(nrToEliminate > this->key_ + 1)
    throw std::invalid_argument("Requested that EliminationTree::eliminatePartial eliminate more variables than exist");
  Conditionals conditionals(nrToEliminate); // reserve a vector of conditional shared pointers
  (void)eliminate_(function, conditionals);  // modify in place
  gttoc(ET_recursive_eliminate);

  // Add conditionals to BayesNet
  gttic(assemble_BayesNet);
  typename BayesNet::shared_ptr bayesNet(new BayesNet);
  BOOST_FOREACH(const typename BayesNet::sharedConditional& conditional, conditionals) {
    if(conditional)
      bayesNet->push_back(conditional);
  }
  gttoc(assemble_BayesNet);

  return bayesNet;
}

/* ************************************************************************* */
template<class FACTOR>
typename EliminationTree<FACTOR>::BayesNet::shared_ptr
EliminationTree<FACTOR>::eliminate(Eliminate function) const {
  size_t nrConditionals = this->key_ + 1;    // root key has highest index
  return eliminatePartial(function, nrConditionals);
}

}
