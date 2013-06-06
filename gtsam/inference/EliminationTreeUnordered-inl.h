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

#include <boost/foreach.hpp>
#include <stack>

#include <gtsam/base/timing.h>
#include <gtsam/inference/EliminationTreeUnordered.h>
#include <gtsam/inference/VariableIndexUnordered.h>

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
    std::vector<sharedNode> nodes(n);
    std::vector<size_t> parents(n, none);
    std::vector<size_t> prevCol(m, none);
    std::vector<bool> factorUsed(m, false);

    try {
      // for column j \in 1 to n do
      for (size_t j = 0; j < n; j++)
      {
        // Retrieve the factors involving this variable and create the current node
        const VariableIndex::Factors& factors = structure[order[j]];
        nodes[j] = boost::make_shared<Node>();
        nodes[j]->key = order[j];

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
              nodes[j]->subTrees.push_back(nodes[r]);
            }
          } else {
            // Add the current factor to the current node since we are at the first variable in this
            // factor.
            nodes[j]->factors.push_back(graph[i]);
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
  template<class BAYESNET, class GRAPH>
  EliminationTreeUnordered<BAYESNET,GRAPH>&
    EliminationTreeUnordered<BAYESNET,GRAPH>::operator=(const EliminationTreeUnordered<BAYESNET,GRAPH>& other)
  {
    // Start by duplicating the roots.
    roots_.clear();
    std::stack<sharedNode, std::vector<sharedNode> > stack;
    BOOST_FOREACH(const sharedNode& root, other.roots_) {
      roots_.push_back(boost::make_shared<Node>(*root));
      stack.push(roots_.back());
    }

    // Traverse the tree, duplicating each node's children and fixing the pointers as we go.  We do
    // not clone any factors, only copy their pointers (this is a standard in GTSAM).
    while(!stack.empty()) {
      sharedNode node = stack.top();
      stack.pop();
      BOOST_FOREACH(sharedNode& child, node->subTrees) {
        // Important: since we are working with a *reference* to a shared pointer in this
        // BOOST_FOREACH loop, the next statement modifies the pointer in the current node's child
        // list - it replaces it with a pointer to a copy of the child.
        child = boost::make_shared<Node>(*child);
        stack.push(child);
      }
    }

    // Assign the remaining factors - these are pointers to factors in the original factor graph and
    // we do not clone them.
    remainingFactors_ = other.remainingFactors_;

    return *this;
  }

  /* ************************************************************************* */
  namespace {
    template<class ELIMINATIONTREE>
    struct EliminationNode {
      bool expanded;
      const typename ELIMINATIONTREE::Node* const treeNode;
      std::vector<typename ELIMINATIONTREE::sharedFactor> factors;
      EliminationNode<ELIMINATIONTREE>* const parent;
      template<typename ITERATOR> EliminationNode(const typename ELIMINATIONTREE::Node* _treeNode, size_t nFactorsToReserve,
        ITERATOR firstFactor, ITERATOR lastFactor, EliminationNode<ELIMINATIONTREE>* _parent) :
      expanded(false), treeNode(_treeNode), parent(_parent) {
        factors.reserve(nFactorsToReserve);
        factors.insert(factors.end(), firstFactor, lastFactor);
      }
    };
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  std::pair<boost::shared_ptr<BAYESNET>, boost::shared_ptr<GRAPH> >
    EliminationTreeUnordered<BAYESNET,GRAPH>::eliminate(Eliminate function) const
  {
    // Stack for eliminating nodes.  We use this stack instead of recursive function calls to
    // avoid call stack overflow due to very long trees that arise from chain-like graphs.  We use
    // an std::vector for storage here since we do not want frequent reallocations and do not care
    // about the vector growing to be very large once and not being deallocated until this
    // function exits, because in the worst case we only store one pointer in this stack for each
    // variable in the system.
    // TODO: Check whether this is faster as a vector (then use indices instead of parent pointers).
    typedef EliminationNode<This> EliminationNode;
    std::stack<EliminationNode, FastList<EliminationNode> > eliminationStack;

    // Create empty Bayes net and factor graph to hold result
    boost::shared_ptr<BayesNetType> bayesNet = boost::make_shared<BayesNetType>();
    // Initialize remaining factors with the factors remaining from creation of the
    // EliminationTree - these are the factors that were not included in the partial elimination
    // at all.
    boost::shared_ptr<FactorGraphType> remainingFactors =
      boost::make_shared<FactorGraphType>(remainingFactors_.begin(), remainingFactors_.end());

    // Add roots to the stack
    BOOST_FOREACH(const sharedNode& root, roots_) {
      eliminationStack.push(
        EliminationNode(root.get(), root->factors.size() + root->subTrees.size(),
        root->factors.begin(), root->factors.end(), 0)); }

    // Until the stack is empty
    while(!eliminationStack.empty()) {
      // Process the next node.  If it has children, add its children to the stack and mark it
      // expanded - we'll come back and eliminate it later after the children have been processed.
      EliminationNode& node = eliminationStack.top();
      if(node.expanded) {
        // Do a dense elimination step
        std::vector<Key> keyAsVector(1); keyAsVector[0] = node.treeNode->key;
        std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<FactorType> > eliminationResult =
          function(node.factors, keyAsVector);

        // Add conditional to BayesNet and remaining factor to parent
        bayesNet->push_back(eliminationResult.first);

        // TODO: Don't add null factor?
        if(node.parent)
          node.parent->factors.push_back(eliminationResult.second);
        else
          remainingFactors->push_back(eliminationResult.second);

        // Remove from stack
        eliminationStack.pop();
      } else {
        // Expand children and mark as expanded
        node.expanded = true;
        BOOST_FOREACH(const sharedNode& child, node.treeNode->subTrees) {
          eliminationStack.push(
            EliminationNode(child.get(), child->factors.size() + child->subTrees.size(),
            child->factors.begin(), child->factors.end(), &node)); }
      }
    }

    // Return results
    return std::make_pair(bayesNet, remainingFactors);
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  void EliminationTreeUnordered<BAYESNET,GRAPH>::print(const std::string& name, const KeyFormatter& formatter) const
  {
    // Depth-first-traversal stack
    std::stack<std::pair<sharedNode, std::string> > stack;

    // Add roots
    BOOST_FOREACH(const sharedNode& node, roots_) { stack.push(std::make_pair(node, "  ")); }

    // Traverse
    while(!stack.empty()) {
      std::pair<sharedNode,string> node_string = stack.top();
      stack.pop();
      std::cout << node_string.second << " (" << formatter(node_string.first->key) << ")\n";
      BOOST_FOREACH(const sharedFactor& factor, node_string.first->factors) {
        if(factor)
          factor->print(node_string.second + "  ");
        else
          std::cout << node_string.second << "  null factor\n";
      }
      BOOST_FOREACH(const sharedNode& child, node_string.first->subTrees) {
        stack.push(std::make_pair(child, node_string.second + "  "));
      }
    }
  }

  /* ************************************************************************* */
  template<class BAYESNET, class GRAPH>
  bool EliminationTreeUnordered<BAYESNET,GRAPH>::equals(const This& expected, double tol) const
  {
    // Depth-first-traversal stacks
    std::stack<sharedNode, std::vector<sharedNode> > stack1, stack2;

    // Add roots in sorted order
    {
      FastMap<Key,sharedNode> keys;
      BOOST_FOREACH(const sharedNode& root, this->roots_) { keys.insert(make_pair(root->key, root)); }
      typedef FastMap<Key,sharedNode>::value_type Key_Node;
      BOOST_FOREACH(const Key_Node& key_node, keys) { stack1.push(key_node.second); }
    }
    {
      FastMap<Key,sharedNode> keys;
      BOOST_FOREACH(const sharedNode& root, expected.roots_) { keys.insert(make_pair(root->key, root)); }
      typedef FastMap<Key,sharedNode>::value_type Key_Node;
      BOOST_FOREACH(const Key_Node& key_node, keys) { stack2.push(key_node.second); }
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
        for(Node::Factors::const_iterator it1 = node1->factors.begin(), it2 = node2->factors.begin();
          it1 != node1->factors.end(); ++it1, ++it2) // Only check it1 == end because we already returned false for different counts
        {
          if(*it1 && *it2) {
            if(!(*it1)->equals(**it2, tol))
              return false;
          } else if(*it1 && !*it2 || *it2 && !*it1) {
            return false;
          }
        }
      }

      // Add children in sorted order
      {
        FastMap<Key,sharedNode> keys;
        BOOST_FOREACH(const sharedNode& node, node1->subTrees) { keys.insert(make_pair(node->key, node)); }
        typedef FastMap<Key,sharedNode>::value_type Key_Node;
        BOOST_FOREACH(const Key_Node& key_node, keys) { stack1.push(key_node.second); }
      }
      {
        FastMap<Key,sharedNode> keys;
        BOOST_FOREACH(const sharedNode& node, node2->subTrees) { keys.insert(make_pair(node->key, node)); }
        typedef FastMap<Key,sharedNode>::value_type Key_Node;
        BOOST_FOREACH(const Key_Node& key_node, keys) { stack2.push(key_node.second); }
      }
    }

    // If either stack is not empty, the number of nodes differed
    if(!stack1.empty() || !stack2.empty())
      return false;

    return true;
  }

}
