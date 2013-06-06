/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation, 
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    inference-inst.h
* @brief   Contains *generic* inference algorithms that convert between templated
* graphical models, i.e., factor graphs, Bayes nets, and Bayes trees
* @author  Frank Dellaert
* @author  Richard Roberts
*/

#pragma once

#include <boost/shared_ptr.hpp>
#include <utility>

#include <gtsam/base/treeTraversal-inst.h>

namespace gtsam {
  namespace inference {

    /* ************************************************************************* */
    namespace {
      template<class ELIMINATIONTREE>
      struct EliminationNode {
        bool expanded;
        const typename ELIMINATIONTREE::Node* const treeNode;
        std::vector<typename ELIMINATIONTREE::sharedFactor> childrenFactors;
        EliminationNode<ELIMINATIONTREE>* const parent;
        EliminationNode(const typename ELIMINATIONTREE::Node* _treeNode, EliminationNode<ELIMINATIONTREE>* _parent) :
          expanded(false), treeNode(_treeNode), parent(_parent) {
            childrenFactors.reserve(treeNode->children.size()); }
      };

      /* ************************************************************************* */
      template<TREE>
      struct EliminationData {
        EliminationData* const parentData;
        std::vector<typename TREE::sharedFactor> childFactors;
        EliminationData(EliminationData* _parentData, size_t nChildren) :
          parentData(_parentData) { childFactors.reserve(nChildren); }
      };

      /* ************************************************************************* */
      template<TREE>
      EliminationData<TREE> eliminationPreOrderVisitor(
        const typename TREE::sharedNode& node, EliminationData<TREE>* parentData)
      {
        // This function is called before visiting the children.  Here, we create this node's data,
        // which includes a pointer to the parent data and space for the factors of the children.
        return EliminationData<TREE>(parentData, node->children.size());
      }

      /* ************************************************************************* */
      template<TREE, RESULT>
      void eliminationPostOrderVisitor(const TREE::Node* const node, EliminationData<TREE>& myData,
        RESULT& result, const typename TREE::Eliminate& eliminationFunction)
      {
        // Call eliminate on the node and add the result to the parent's gathered factors
        myData.parentData->childFactors.push_back(node->eliminate(result, eliminationFunction, myData.childFactors));
      }
    }

    /* ************************************************************************* */
    /** Eliminate an elimination tree or a Bayes tree (used internally).  Requires
     *  TREE::BayesNetType, TREE::FactorGraphType, TREE::sharedConditional, TREE::sharedFactor,
     *  TREE::Node, TREE::sharedNode, TREE::Node::factors, TREE::Node::children. */
    template<class TREE, class RESULT>
    std::vector<typename TREE::sharedFactor>
    EliminateTree(RESULT& result, const TREE& tree, const typename TREE::Eliminate& function)
    {
      // Typedefs
      typedef typename TREE::sharedNode sharedNode;
      typedef typename TREE::sharedFactor sharedFactor;

      // Allocate remaining factors
      std::vector<sharedFactor> remainingFactors;
      remainingFactors.reserve(tree.roots().size());

      treeTraversal::DepthFirstForest(tree, remainingFactors, )

      // Stack for eliminating nodes.  We use this stack instead of recursive function calls to
      // avoid call stack overflow due to very long trees that arise from chain-like graphs.
      // TODO: Check whether this is faster as a vector (then use indices instead of parent pointers).
      typedef EliminationNode<TREE> EliminationNode;
      std::stack<EliminationNode, FastList<EliminationNode> > eliminationStack;

      // Allocate remaining factors
      std::vector<sharedFactor> remainingFactors;
      remainingFactors.reserve(tree.roots().size());

      // Add roots to the stack (use reverse foreach so conditionals to appear in elimination order -
      // doesn't matter for computation but can make printouts easier to interpret by hand).
      BOOST_REVERSE_FOREACH(const sharedNode& root, tree.roots()) {
        eliminationStack.push(
          EliminationNode(root.get(), 0)); }

      // Until the stack is empty
      while(!eliminationStack.empty()) {
        // Process the next node.  If it has children, add its children to the stack and mark it
        // expanded - we'll come back and eliminate it later after the children have been processed.
        EliminationNode& node = eliminationStack.top();
        if(node.expanded)
        {
          // Do elimination step
          sharedFactor remainingFactor = node.treeNode->eliminate(result, function, node.childrenFactors);

          // TODO: Don't add null factor?
          if(node.parent)
            node.parent->childrenFactors.push_back(remainingFactor);
          else
            remainingFactors.push_back(remainingFactor);

          // Remove from stack
          eliminationStack.pop();
        } else
        {
          // Expand children and mark as expanded (use reverse foreach so conditionals to appear in
          // elimination order - doesn't matter for computation but can make printouts easier to
          // interpret by hand).
          node.expanded = true;
          BOOST_REVERSE_FOREACH(const sharedNode& child, node.treeNode->children) {
            eliminationStack.push(
              EliminationNode(child.get(), &node)); }
        }
      }

      // Return remaining factors
      return remainingFactors;
    }

  }
}