/**
 * @file    EliminationTree.cpp
 * @brief   
 * @author  Frank Dellaert
 * @created Oct 13, 2010
 */
#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/FactorGraph-inl.h>

#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <set>
#include <vector>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
template<class FACTOR>
typename EliminationTree<FACTOR>::sharedFactor
EliminationTree<FACTOR>::eliminate_(Conditionals& conditionals) const {

  static const bool debug = false;

  if(debug) cout << "ETree: eliminating " << this->key_ << endl;

  FastSet<Index> separator;

  // Create the list of factors to be eliminated, initially empty, and reserve space
  FactorGraph<FACTOR> factors;
  factors.reserve(this->factors_.size() + this->subTrees_.size());

  // Add all factors associated with the current node
  factors.push_back(this->factors_.begin(), this->factors_.end());

  // for all subtrees, eliminate into Bayes net and a separator factor, added to [factors]
  BOOST_FOREACH(const shared_ptr& child, subTrees_) {
    factors.push_back(child->eliminate_(conditionals)); }

  // Combine all factors (from this node and from subtrees) into a joint factor
  pair<typename BayesNet::shared_ptr, typename FACTOR::shared_ptr> eliminated(
      FACTOR::CombineAndEliminate(factors, 1));
  conditionals[this->key_] = eliminated.first->front();

  return eliminated.second;
}

/* ************************************************************************* */
template<class FACTOR>
vector<Index> EliminationTree<FACTOR>::ComputeParents(const VariableIndex& structure) {

  // Number of factors and variables
  const size_t m = structure.nFactors();
  const size_t n = structure.size();

  static const Index none = numeric_limits<Index>::max();

  // Allocate result parent vector and vector of last factor columns
  vector<Index> parents(n, none);
  vector<Index> prevCol(m, none);

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
typename EliminationTree<FACTOR>::shared_ptr
EliminationTree<FACTOR>::Create(const FactorGraph<DERIVEDFACTOR>& factorGraph, const VariableIndex& structure) {

  static const bool debug = false;

  tic(1, "ET ComputeParents");
  // Compute the tree structure
  vector<Index> parents(ComputeParents(structure));
  toc(1, "ET ComputeParents");

  // Number of variables
  const size_t n = structure.size();

  static const Index none = numeric_limits<Index>::max();

  // Create tree structure
  tic(2, "assemble tree");
  vector<shared_ptr> trees(n);
  for (Index k = 1; k <= n; k++) {
    Index j = n - k;
    trees[j].reset(new EliminationTree(j));
    if (parents[j] != none)
      trees[parents[j]]->add(trees[j]);
  }
  toc(2, "assemble tree");

  // Hang factors in right places
  tic(3, "hang factors");
  BOOST_FOREACH(const typename DERIVEDFACTOR::shared_ptr& derivedFactor, factorGraph) {
    // Here we static_cast to the factor type of this EliminationTree.  This
    // allows performing symbolic elimination on, for example, GaussianFactors.
    sharedFactor factor(boost::shared_static_cast<FACTOR>(derivedFactor));
    Index j = factor->front();
    trees[j]->add(factor);
  }
  toc(3, "hang factors");

  // Assert that all other nodes have parents, i.e. that this is not a forest.
#ifndef NDEBUG
  for(typename vector<shared_ptr>::const_iterator tree=trees.begin(); tree!=trees.end()-1; ++tree)
    assert((*tree) != shared_ptr());
#endif

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
void EliminationTree<FACTORGRAPH>::print(const std::string& name) const {
  cout << name << " (" << key_ << ")" << endl;
  BOOST_FOREACH(const sharedFactor& factor, factors_) {
    factor->print(name + "  "); }
  BOOST_FOREACH(const shared_ptr& child, subTrees_) {
    child->print(name + "  "); }
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
EliminationTree<FACTOR>::eliminate() const {

  // call recursive routine
  tic(1, "ET recursive eliminate");
  Conditionals conditionals(this->key_ + 1);
  (void)eliminate_(conditionals);
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
