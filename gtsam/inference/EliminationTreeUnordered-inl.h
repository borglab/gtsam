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
* @author  Richard Roberts
* @date    Oct 13, 2010
*/
#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/inference/EliminationTreeUnordered.h>

#include <boost/foreach.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  EliminationTreeUnordered<BAYESNET,GRAPH>::EliminationTreeUnordered(const FactorGraphType& graph,
    const VariableIndexUnordered& structure, const std::vector<Key>& order)
  {
    gttic(ET_Create1);

    // Number of factors and variables - NOTE in the case of partial elimination, n here may
    // be fewer variables than are actually present in the graph.
    const size_t m = graph.size();
    const size_t n = order.size();

    static const size_t none = std::numeric_limits<size_t>::max();

    // Allocate result parent vector and vector of last factor columns
    std::vector<shared_ptr> nodes(n);
    std::vector<size_t> parents(n, none);
    std::vector<size_t> prevCol(m, none);
    std::vector<bool> factorUsed(m, false);

    try {
      // for column j \in 1 to n do
      for (size_t j = 0; j < n; j++)
      {
        // Retrieve the factors involving this variable and create the current node
        const VariableIndex::Factors& factors = structure[order[j]];
        nodes[j] = boost::make_shared<EliminationTreeUnordered<FACTOR> >(order[j]);

        // for row i \in Struct[A*j] do
        BOOST_FOREACH(const size_t i, factors) {
          // If we already hit a variable in this factor, make the subtree containing the previous
          // variable in this factor a child of the current node.  This means that the variables
          // eliminated earlier in the factor depend on the later variables in the factor.  If we
          // haven't yet hit a variable in this factor, we add the factor to the current node.
          // TODO: Store root shortcuts instead of parents.
          if (prevCol[i] != none) {
            size_t k = prevCol[i];
            // Find root r of the current tree that contains k. Use raw pointers in computing the
            // parents to avoid changing the reference counts while traversing up the tree.
            size_t r = k;
            while (parents[r] != none)
              r = parents[r];
            // If the root of the subtree involving this node is actually the current node,
            // TODO: what does this mean?  forest?
            if (r != j) {
              // Now that we found the root, hook up parent and child pointers in the nodes.
              parents[r] = j;
              nodes[j]->subTrees_.push_back(nodes[r]);
            }
          } else {
            // Add the current factor to the current node since we are at the first variable in this
            // factor.
            nodes[j]->factors_.push_back(graph[i]);
            factorUsed[i] = true;
          }
          prevCol[i] = j;
        }
      }
    } catch(std::invalid_argument& e) {
      // If this is thrown from structure[order[j]] above, it means that it was requested to
      // eliminate a variable not present in the graph, so throw a more informative error message.
      throw std::invalid_argument("EliminationTree: given ordering contains variables that are not involved in the factor graph");
    } catch(...) {
      throw;
    }

    // Find roots
    assert(parents.back() == none); // We expect the last-eliminated node to be a root no matter what
    for(size_t j = 0; j < n; ++j)
      if(parents[j] == none)
        roots_.push_back(nodes[j]);

    // Gather remaining factors
    for(size_t i = 0; i < m; ++i)
      if(!factorUsed[i])
        remainingFactors_.push_back(graph[i]);
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  EliminationTreeUnordered<BAYESNET,GRAPH>::EliminationTreeUnordered(
    const FactorGraphType& factorGraph, const std::vector<Key>& order)
  {
    gttic(ET_Create2);
    // Build variable index first
    const VariableIndexUnordered variableIndex(factorGraph);
    This temp(factorGraph, variableIndex, order);
    roots_.swap(temp.roots_); // Swap in the tree, and temp will be deleted
    remainingFactors_.swap(temp.remainingFactors_);
  }

  /* ************************************************************************* */
  namespace {
    template<class FACTOR>
    struct EliminationNode {
      bool expanded;
      Key key;
      std::vector<boost::shared_ptr<FACTOR> > factors;
      EliminationNode<FACTOR>* parent;
      template<typename ITERATOR> EliminationNode(
        Key _key, size_t nFactorsToReserve, ITERATOR firstFactor, ITERATOR lastFactor, EliminationNode<FACTOR>* _parent) :
      expanded(false), key(_key), parent(_parent) {
        factors.reserve(nFactorsToReserve);
        factors.insert(factors.end(), firstFactor, lastFactor);
      }
    };
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  std::pair<boost::shared_ptr<BAYESNET>, boost::shared_ptr<GRAPH> >
    EliminationTreeUnordered<BAYESNET,GRAPH>::eliminate(Eliminate function)
  {
    // Stack for eliminating nodes.  We use this stack instead of recursive function calls to
    // avoid call stack overflow due to very long trees that arise from chain-like graphs.  We use
    // an std::vector for storage here since we do not want frequent reallocations and do not care
    // about the vector growing to be very large once and not being deallocated until this
    // function exits, because in the worst case we only store one pointer in this stack for each
    // variable in the system.
    typedef EliminationNode<FactorType> EliminationNode;
    std::stack<EliminationNode, std::vector<EliminationNode> > eliminationStack;

    // Create empty Bayes net and factor graph to hold result
    boost::shared_ptr<BayesNetType> bayesNet = boost::make_shared<BayesNetType>();
    // Initialize remaining factors with the factors remaining from creation of the
    // EliminationTree - these are the factors that were not included in the partial elimination
    // at all.
    boost::shared_ptr<FactorGraphType> remainingFactors =
      boost::make_shared<FactorGraphType>(remainingFactors_);

    // Add roots to the stack
    BOOST_FOREACH(const sharedNode& root, roots_) {
      eliminationStack.push(
        EliminationNode(root->key, root->factors.size() + root->subTrees.size(),
        root->factors.begin(), root->factors.end(), 0)); }

    // Until the stack is empty
    while(!eliminationStack.empty()) {
      // Process the next node.  If it has children, add its children to the stack and mark it
      // expanded - we'll come back and eliminate it later after the children have been processed.
      EliminationNode& node = nodeStack.top();
      if(node.expanded) {
        // Remove from stack
        nodeStack.pop();

        // Do a dense elimination step
        std::vector<Key> keyAsVector(1); keyAsVector[0] = node.key;
        std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<FactorType> > eliminationResult =
          function(node.factors, keyAsVector);

        // Add conditional to BayesNet and remaining factor to parent
        bayesNet->push_back(eliminationResult.first);

        // TODO: Don't add null factor?
        if(node.parent)
          node.parent->factors.push_back(eliminationResult.second);
        else
          remainingFactors->push_back(eliminationResult.second);
      } else {
        // Expand children and mark as expanded
        node.expanded = true;
        BOOST_FOREACH(const sharedNode& child, node.subTrees) {
          nodeStack.push(
            EliminationNode(child->key, child->factors.size() + child->subTrees.size(),
            child->factors.begin(), child->factors.end(), 0)); }
      }
    }

    // Return results
    return std::make_pair(bayesNet, remainingFactors);
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  void EliminationTreeUnordered<BAYESNET,GRAPH>::print(const std::string& name,
    const IndexFormatter& formatter) const {
      std::cout << name << " (" << formatter(key_) << ")" << std::endl;
      BOOST_FOREACH(const sharedFactor& factor, factors_) {
        factor->print(name + "  ", formatter); }
      BOOST_FOREACH(const shared_ptr& child, subTrees_) {
        child->print(name + "  ", formatter); }
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  bool EliminationTreeUnordered<BAYESNET,GRAPH>::equals(const This& expected, double tol) const {
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

}
