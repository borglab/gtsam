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

  tic(1, "ET ComputeParents");
  // Compute the tree structure
  std::vector<Index> parents(ComputeParents(structure));
  toc(1, "ET ComputeParents");

  // Number of variables
  const size_t n = structure.size();

  static const Index none = std::numeric_limits<Index>::max();

  // Create tree structure
  tic(2, "assemble tree");
  std::vector<shared_ptr> trees(n);
  for (Index k = 1; k <= n; k++) {
    Index j = n - k;  // Start at the last variable and loop down to 0
    trees[j].reset(new EliminationTree(j));  // Create a new node on this variable
    if (parents[j] != none)  // If this node has a parent, add it to the parent's children
      trees[parents[j]]->add(trees[j]);
    else if(!structure[j].empty() && j != n - 1) // If a node other than the last has no parents, this is a forest
      throw DisconnectedGraphException();
  }
  toc(2, "assemble tree");

  // Hang factors in right places
  tic(3, "hang factors");
  BOOST_FOREACH(const typename boost::shared_ptr<DERIVEDFACTOR>& derivedFactor, factorGraph) {
    // Here we upwards-cast to the factor type of this EliminationTree.  This
    // allows performing symbolic elimination on, for example, GaussianFactors.
    if(derivedFactor) {
      sharedFactor factor(derivedFactor);
      Index j = *std::min_element(factor->begin(), factor->end());
      trees[j]->add(factor);
    }
  }
  toc(3, "hang factors");

  if(debug)
    trees.back()->print("ETree: ");

  return trees.back();
}

/* ************************************************************************* */
template<class FACTOR>
template<class DERIVEDFACTOR>
typename EliminationTree<FACTOR>::shared_ptr
EliminationTree<FACTOR>::Create(const FactorGraph<DERIVEDFACTOR>& factorGraph) {

  // Build variable index
  tic(0, "ET Create, variable index");
  const VariableIndex variableIndex(factorGraph);
  toc(0, "ET Create, variable index");

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
EliminationTree<FACTOR>::eliminate(Eliminate function) const {

  // call recursive routine
  tic(1, "ET recursive eliminate");
  size_t nrConditionals = this->key_ + 1;    // root key has highest index
  Conditionals conditionals(nrConditionals); // reserve a vector of conditional shared pointers
  (void)eliminate_(function, conditionals);  // modify in place
  toc(1, "ET recursive eliminate");

  // Add conditionals to BayesNet
  tic(2, "assemble BayesNet");
  typename BayesNet::shared_ptr bayesNet(new BayesNet);
  BOOST_FOREACH(const typename BayesNet::sharedConditional& conditional, conditionals) {
    if(conditional)
      bayesNet->push_back(conditional);
  }
  toc(2, "assemble BayesNet");

  return bayesNet;
}

}
