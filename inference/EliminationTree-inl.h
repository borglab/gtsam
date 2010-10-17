/**
 * @file    EliminationTree.cpp
 * @brief   
 * @author  Frank Dellaert
 * @created Oct 13, 2010
 */

#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/VariableSlots-inl.h>

#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <set>
#include <vector>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
template<class FACTORGRAPH>
typename EliminationTree<FACTORGRAPH>::EliminationResult
EliminationTree<FACTORGRAPH>::eliminate_() const {

  typename FACTORGRAPH::bayesnet_type bayesNet;

  set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > separator;

  // Create the list of factors to be eliminated
  FACTORGRAPH factors;
  factors.reserve(this->factors_.size() + this->subTrees_.size());

  // add all factors associated with root
  factors.push_back(this->factors_.begin(), this->factors_.end());

  // for all children, eliminate into Bayes net and add the eliminated factors
  BOOST_FOREACH(const shared_ptr& child, subTrees_) {
    EliminationResult result = child->eliminate_();
    bayesNet.push_back(result.first);
    factors.push_back(result.second);
  }

  // eliminate the joint factor and add the conditional to the bayes net
  typename FACTORGRAPH::sharedFactor jointFactor(FACTORGRAPH::Factor::Combine(factors, VariableSlots(factors)));
  bayesNet.push_back(jointFactor->eliminateFirst());

  return EliminationResult(bayesNet, jointFactor);
}

/* ************************************************************************* */
template<class FACTORGRAPH>
vector<Index> EliminationTree<FACTORGRAPH>::ComputeParents(const VariableIndex<>& structure) {

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
    BOOST_FOREACH(typename VariableIndex<>::mapped_factor_type i_pos, structure[j]) {
      const size_t i = i_pos.factorIndex;
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
template<class FACTORGRAPH>
typename EliminationTree<FACTORGRAPH>::shared_ptr
EliminationTree<FACTORGRAPH>::Create(const FACTORGRAPH& factorGraph) {

  // Create column structure
  VariableIndex<> varIndex(factorGraph);

  // Compute the tree structure
  vector<Index> parents(ComputeParents(varIndex));

  // Number of variables
  const size_t n = varIndex.size();

  static const Index none = numeric_limits<Index>::max();

  // Create tree structure
  vector<shared_ptr> trees(n);
  for (Index k = 1; k <= n; k++) {
    Index j = n - k;
    trees[j].reset(new EliminationTree(j));
    if (parents[j] != none)
      trees[parents[j]]->add(trees[j]);
  }

  // Hang factors in right places
  BOOST_FOREACH(const sharedFactor& factor, factorGraph) {
    Index j = factor->front();
    trees[j]->add(factor);
  }

  // Assert that all other nodes have parents, i.e. that this is not a forest.
#ifndef NDEBUG
  for(typename vector<shared_ptr>::const_iterator tree=trees.begin(); tree!=trees.end()-1; ++tree)
    assert((*tree) != shared_ptr());
#endif

  return trees.back();
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
template<class FACTORGRAPH>
typename FACTORGRAPH::bayesnet_type::shared_ptr EliminationTree<FACTORGRAPH>::eliminate() const {

  // call recursive routine
  EliminationResult result = eliminate_();
  return typename FACTORGRAPH::bayesnet_type::shared_ptr(new typename FACTORGRAPH::bayesnet_type(result.first));
}

}
