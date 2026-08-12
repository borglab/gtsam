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

#include <utility>

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/base/FastVector.h>

namespace gtsam {
  namespace inference {

    namespace {
      /* ************************************************************************* */
      template<class TREE>
      struct EliminationData {
        EliminationData* const parentData;
        FastVector<typename TREE::sharedFactor> childFactors;
        EliminationData(EliminationData* _parentData, size_t nChildren) :
          parentData(_parentData) { childFactors.reserve(nChildren); }
      };

      /* ************************************************************************* */
      template<class TREE>
      EliminationData<TREE> eliminationPreOrderVisitor(
        const typename TREE::sharedNode& node, EliminationData<TREE>& parentData)
      {
        // This function is called before visiting the children.  Here, we create this node's data,
        // which includes a pointer to the parent data and space for the factors of the children.
        return EliminationData<TREE>(&parentData, node->children.size());
      }

      /* ************************************************************************* */
      template<class TREE, class RESULT>
      struct EliminationPostOrderVisitor
      {
        RESULT& result;
        const typename TREE::Eliminate& eliminationFunction;
        EliminationPostOrderVisitor(RESULT& result, const typename TREE::Eliminate& eliminationFunction) :
          result(result), eliminationFunction(eliminationFunction) {}
        void operator()(const typename TREE::sharedNode& node, EliminationData<TREE>& myData)
        {
          // Call eliminate on the node and add the result to the parent's gathered factors
          typename TREE::sharedFactor childFactor = node->eliminate(result, eliminationFunction, myData.childFactors);
          if(childFactor && !childFactor->empty())
            myData.parentData->childFactors.push_back(childFactor);
        }
      };
    }

    /* ************************************************************************* */
    /** Eliminate an elimination tree or a Bayes tree (used internally).  Requires
     *  TREE::BayesNetType, TREE::FactorGraphType, TREE::sharedConditional, TREE::sharedFactor,
     *  TREE::Node, TREE::sharedNode, TREE::Node::factors, TREE::Node::children. */
    template<class TREE, class RESULT>
    FastVector<typename TREE::sharedFactor>
    EliminateTree(RESULT& result, const TREE& tree, const typename TREE::Eliminate& function)
    {
      // Do elimination using a depth-first traversal.  During the pre-order visit (see
      // eliminationPreOrderVisitor), we store a pointer to the parent data (where we'll put the
      // remaining factor) and reserve a vector of factors to store the children elimination
      // results.  During the post-order visit (see eliminationPostOrderVisitor), we call dense
      // elimination (using the gathered child factors) and store the result in the parent's
      // gathered factors.
      EliminationData<TREE> rootData(0, tree.roots().size());
      EliminationPostOrderVisitor<TREE,RESULT> visitorPost(result, function);
      treeTraversal::DepthFirstForest(tree, rootData, eliminationPreOrderVisitor<TREE>, visitorPost);

      // Return remaining factors
      return rootData.childFactors;
    }

  }
}
