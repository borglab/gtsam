/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file JunctionTree-inst.h
 * @date Feb 4, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @author Richard Roberts
 * @brief The junction tree, template bodies
 */

#pragma once

#include <gtsam/inference/JunctionTree.h>
#include <gtsam/inference/ClusterTree-inst.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/SymbolicFactor-inst.h>

namespace gtsam {

template<class BAYESTREE, class GRAPH, class ETREE_NODE>
struct ConstructorTraversalData {
  typedef typename JunctionTree<BAYESTREE, GRAPH>::Node Node;
  typedef typename JunctionTree<BAYESTREE, GRAPH>::sharedNode sharedNode;

  ConstructorTraversalData* const parentData;
  sharedNode junctionTreeNode;
  FastVector<SymbolicConditional::shared_ptr> childSymbolicConditionals;
  FastVector<SymbolicFactor::shared_ptr> childSymbolicFactors;

  // Small inner class to store symbolic factors
  class SymbolicFactors: public FactorGraph<Factor> {
  };

  ConstructorTraversalData(ConstructorTraversalData* _parentData) :
      parentData(_parentData) {
  }

  // Pre-order visitor function
  static ConstructorTraversalData ConstructorTraversalVisitorPre(
      const std::shared_ptr<ETREE_NODE>& node,
      ConstructorTraversalData& parentData) {
    // On the pre-order pass, before children have been visited, we just set up
    // a traversal data structure with its own JT node, and create a child
    // pointer in its parent.
    ConstructorTraversalData myData = ConstructorTraversalData(&parentData);
    myData.junctionTreeNode =
        std::make_shared<Node>(node->key, node->factors);
    parentData.junctionTreeNode->addChild(myData.junctionTreeNode);
    return myData;
  }

  // Post-order visitor function
  static void ConstructorTraversalVisitorPostAlg2(
      const std::shared_ptr<ETREE_NODE>& ETreeNode,
      const ConstructorTraversalData& myData) {
    // In this post-order visitor, we combine the symbolic elimination results
    // from the elimination tree children and symbolically eliminate the current
    // elimination tree node.  We then check whether each of our elimination
    // tree child nodes should be merged with us.  The check for this is that
    // our number of symbolic elimination parents is exactly 1 less than
    // our child's symbolic elimination parents - this condition indicates that
    // eliminating the current node did not introduce any parents beyond those
    // already in the child->

    // Do symbolic elimination for this node
    SymbolicFactors symbolicFactors;
    symbolicFactors.reserve(
        ETreeNode->factors.size() + myData.childSymbolicFactors.size());
    // Add ETree node factors
    symbolicFactors += ETreeNode->factors;
    // Add symbolic factors passed up from children
    symbolicFactors += myData.childSymbolicFactors;

    Ordering keyAsOrdering;
    keyAsOrdering.push_back(ETreeNode->key);
    const auto [myConditional, mySeparatorFactor] =
        internal::EliminateSymbolic(symbolicFactors, keyAsOrdering);

    // Store symbolic elimination results in the parent
    myData.parentData->childSymbolicConditionals.push_back(myConditional);
    myData.parentData->childSymbolicFactors.push_back(mySeparatorFactor);

    sharedNode node = myData.junctionTreeNode;
    const FastVector<SymbolicConditional::shared_ptr>& childConditionals =
        myData.childSymbolicConditionals;
    node->problemSize_ = (int) (myConditional->size() * symbolicFactors.size());

    // Merge our children if they are in our clique - if our conditional has
    // exactly one fewer parent than our child's conditional.
    const size_t myNrParents = myConditional->nrParents();
    const size_t nrChildren = node->nrChildren();
    assert(childConditionals.size() == nrChildren);

    // decide which children to merge, as index into children
    std::vector<size_t> nrFrontals = node->nrFrontalsOfChildren();
    std::vector<bool> merge(nrChildren, false);
    size_t myNrFrontals = 1;
    for (size_t i = 0;i<nrChildren;i++){
      // Check if we should merge the i^th child
      if (myNrParents + myNrFrontals == childConditionals[i]->nrParents()) {
        // Increment number of frontal variables
        myNrFrontals += nrFrontals[i];
        merge[i] = true;
      }
    }

    // now really merge
    node->mergeChildren(merge);
  }
};

/* ************************************************************************* */
template<class BAYESTREE, class GRAPH>
template<class ETREE_BAYESNET, class ETREE_GRAPH>
JunctionTree<BAYESTREE, GRAPH>::JunctionTree(
    const EliminationTree<ETREE_BAYESNET, ETREE_GRAPH>& eliminationTree) {
  gttic(JunctionTree_FromEliminationTree);
  // Here we rely on the BayesNet having been produced by this elimination tree,
  // such that the conditionals are arranged in DFS post-order.  We traverse the
  // elimination tree, and inspect the symbolic conditional corresponding to
  // each node.  The elimination tree node is added to the same clique with its
  // parent if it has exactly one more Bayes net conditional parent than
  // does its elimination tree parent.

  // Traverse the elimination tree, doing symbolic elimination and merging nodes
  // as we go.  Gather the created junction tree roots in a dummy Node.
  typedef typename EliminationTree<ETREE_BAYESNET, ETREE_GRAPH>::Node ETreeNode;
  typedef ConstructorTraversalData<BAYESTREE, GRAPH, ETreeNode> Data;
  Data rootData(0);
  // Make a dummy node to gather the junction tree roots
  rootData.junctionTreeNode = std::make_shared<typename Base::Node>();
  treeTraversal::DepthFirstForest(eliminationTree, rootData,
      Data::ConstructorTraversalVisitorPre,
      Data::ConstructorTraversalVisitorPostAlg2);

  // Assign roots from the dummy node
  this->addChildrenAsRoots(rootData.junctionTreeNode);

  // Transfer remaining factors from elimination tree
  Base::remainingFactors_ = eliminationTree.remainingFactors();
}

} // namespace gtsam
