/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    EliminationTree-inst.h
* @author  Frank Dellaert
* @author  Richard Roberts
* @date    Oct 13, 2010
*/
#pragma once

#include <stack>

#include <gtsam/base/timing.h>
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/inference-inst.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  typename EliminationTree<BAYESNET,GRAPH>::sharedFactor
    EliminationTree<BAYESNET,GRAPH>::Node::eliminate(
    const std::shared_ptr<BayesNetType>& output,
    const Eliminate& function, const FastVector<sharedFactor>& childrenResults) const
  {
    // This function eliminates one node (Node::eliminate) - see below eliminate for the whole tree.

    assert(childrenResults.size() == children.size());

    // Gather factors
    FactorGraphType gatheredFactors;
    gatheredFactors.reserve(factors.size() + children.size());
    gatheredFactors.push_back(factors.begin(), factors.end());
    gatheredFactors.push_back(childrenResults.begin(), childrenResults.end());

    // Do dense elimination step
    KeyVector keyAsVector(1); keyAsVector[0] = key;
    auto eliminationResult = function(gatheredFactors, Ordering(keyAsVector));

    // Add conditional to BayesNet
    output->push_back(eliminationResult.first);

    // Return result
    return eliminationResult.second;
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  void EliminationTree<BAYESNET,GRAPH>::Node::print(
    const std::string& str, const KeyFormatter& keyFormatter) const
  {
    std::cout << str << "(" << keyFormatter(key) << ")\n";
    for(const sharedFactor& factor: factors) {
      if(factor)
        factor->print(str);
      else
        std::cout << str << "null factor\n";
    }
  }


  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  EliminationTree<BAYESNET,GRAPH>::EliminationTree(const FactorGraphType& graph,
    const VariableIndex& structure, const Ordering& order)
  {
    gttic(EliminationTree_Contructor);

    // Number of factors and variables - NOTE in the case of partial elimination, n here may
    // be fewer variables than are actually present in the graph.
    const size_t m = graph.size();
    const size_t n = order.size();

    static const size_t none = std::numeric_limits<size_t>::max();

    // Allocate result parent vector and vector of last factor columns
    FastVector<sharedNode> nodes(n);
    FastVector<size_t> parents(n, none);
    FastVector<size_t> prevCol(m, none);
    FastVector<bool> factorUsed(m, false);

    try {
      // for column j \in 1 to n do
      for (size_t j = 0; j < n; j++)
      {
        // Retrieve the factors involving this variable and create the current node
        const FactorIndices& factors = structure[order[j]];
        const sharedNode node = std::make_shared<Node>();
        node->key = order[j];

        // for row i \in Struct[A*j] do
        node->children.reserve(factors.size());
        node->factors.reserve(factors.size());
        for(const size_t i: factors) {
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
              node->children.push_back(nodes[r]);
            }
          } else {
            // Add the factor to the current node since we are at the first variable in this factor.
            node->factors.push_back(graph[i]);
            factorUsed[i] = true;
          }
          prevCol[i] = j;
        }
        nodes[j] = node;
      }
    } catch(std::invalid_argument& e) {
      // If this is thrown from structure[order[j]] above, it means that it was requested to
      // eliminate a variable not present in the graph, so throw a more informative error message.
      (void)e; // Prevent unused variable warning
      throw std::invalid_argument("EliminationTree: given ordering contains variables that are not involved in the factor graph");
    } catch(...) {
      throw;
    }

    // Find roots
    assert(parents.empty() || parents.back() == none); // We expect the last-eliminated node to be a root no matter what
    for(size_t j = 0; j < n; ++j)
      if(parents[j] == none)
        roots_.push_back(nodes[j]);

    // Gather remaining factors (exclude null factors)
    for(size_t i = 0; i < m; ++i)
      if(!factorUsed[i] && graph[i])
        remainingFactors_.push_back(graph[i]);
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  EliminationTree<BAYESNET,GRAPH>::EliminationTree(
    const FactorGraphType& factorGraph, const Ordering& order)
  {
    gttic(ET_Create2);
    // Build variable index first
    const VariableIndex variableIndex(factorGraph);
    This temp(factorGraph, variableIndex, order);
    this->swap(temp); // Swap in the tree, and temp will be deleted
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  EliminationTree<BAYESNET,GRAPH>&
    EliminationTree<BAYESNET,GRAPH>::operator=(const EliminationTree<BAYESNET,GRAPH>& other)
  {
    // Start by duplicating the tree.
    roots_ = treeTraversal::CloneForest(other);

    // Assign the remaining factors - these are pointers to factors in the original factor graph and
    // we do not clone them.
    remainingFactors_ = other.remainingFactors_;

    return *this;
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  std::pair<std::shared_ptr<BAYESNET>, std::shared_ptr<GRAPH> >
    EliminationTree<BAYESNET,GRAPH>::eliminate(Eliminate function) const
  {
    gttic(EliminationTree_eliminate);
    // Allocate result
    auto result = std::make_shared<BayesNetType>();

    // Run tree elimination algorithm
    FastVector<sharedFactor> remainingFactors = inference::EliminateTree(result, *this, function);

    // Add remaining factors that were not involved with eliminated variables
    auto allRemainingFactors = std::make_shared<FactorGraphType>();
    allRemainingFactors->push_back(remainingFactors_.begin(), remainingFactors_.end());
    allRemainingFactors->push_back(remainingFactors.begin(), remainingFactors.end());

    // Return result
    return std::make_pair(result, allRemainingFactors);
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  void EliminationTree<BAYESNET,GRAPH>::print(const std::string& name, const KeyFormatter& formatter) const
  {
    treeTraversal::PrintForest(*this, name, formatter);
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  bool EliminationTree<BAYESNET,GRAPH>::equals(const This& expected, double tol) const
  {
    // Depth-first-traversal stacks
    std::stack<sharedNode, FastVector<sharedNode> > stack1, stack2;

    // Add roots in sorted order
    {
      FastMap<Key,sharedNode> keys;
      for(const sharedNode& root: this->roots_) { keys.insert(std::make_pair(root->key, root)); }
      typedef typename FastMap<Key,sharedNode>::value_type Key_Node;
      for(const Key_Node& key_node: keys) { stack1.push(key_node.second); }
    }
    {
      FastMap<Key,sharedNode> keys;
      for(const sharedNode& root: expected.roots_) { keys.insert(std::make_pair(root->key, root)); }
      typedef typename FastMap<Key,sharedNode>::value_type Key_Node;
      for(const Key_Node& key_node: keys) { stack2.push(key_node.second); }
    }

    // Traverse, adding children in sorted order
    while(!stack1.empty() && !stack2.empty()) {
      // Pop nodes
      sharedNode node1 = stack1.top();
      stack1.pop();
      sharedNode node2 = stack2.top();
      stack2.pop();

      // Compare nodes
      if(node1->key != node2->key)
        return false;
      if(node1->factors.size() != node2->factors.size()) {
        return false;
      } else {
        for(typename Node::Factors::const_iterator it1 = node1->factors.begin(), it2 = node2->factors.begin();
          it1 != node1->factors.end(); ++it1, ++it2) // Only check it1 == end because we already returned false for different counts
        {
          if(*it1 && *it2) {
            if(!(*it1)->equals(**it2, tol))
              return false;
          } else if((*it1 && !*it2) || (*it2 && !*it1)) {
            return false;
          }
        }
      }

      // Add children in sorted order
      {
        FastMap<Key,sharedNode> keys;
        for(const sharedNode& node: node1->children) { keys.insert(std::make_pair(node->key, node)); }
        typedef typename FastMap<Key,sharedNode>::value_type Key_Node;
        for(const Key_Node& key_node: keys) { stack1.push(key_node.second); }
      }
      {
        FastMap<Key,sharedNode> keys;
        for(const sharedNode& node: node2->children) { keys.insert(std::make_pair(node->key, node)); }
        typedef typename FastMap<Key,sharedNode>::value_type Key_Node;
        for(const Key_Node& key_node: keys) { stack2.push(key_node.second); }
      }
    }

    // If either stack is not empty, the number of nodes differed
    if(!stack1.empty() || !stack2.empty())
      return false;

    return true;
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  void EliminationTree<BAYESNET,GRAPH>::swap(This& other) {
    roots_.swap(other.roots_);
    remainingFactors_.swap(other.remainingFactors_);
  }


}
