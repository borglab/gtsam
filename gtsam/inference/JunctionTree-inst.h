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
  sharedNode myJTNode;
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
      const boost::shared_ptr<ETREE_NODE>& node,
      ConstructorTraversalData& parentData) {
    // On the pre-order pass, before children have been visited, we just set up
    // a traversal data structure with its own JT node, and create a child
    // pointer in its parent.
    ConstructorTraversalData myData = ConstructorTraversalData(&parentData);
    myData.myJTNode = boost::make_shared<Node>(node->key, node->factors);
    parentData.myJTNode->children.push_back(myData.myJTNode);
    return myData;
  }

  // Post-order visitor function
  static void ConstructorTraversalVisitorPostAlg2(
      const boost::shared_ptr<ETREE_NODE>& ETreeNode,
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
    symbolicFactors.reserve(ETreeNode->factors.size() + myData.childSymbolicFactors.size());
    // Add ETree node factors
    symbolicFactors += ETreeNode->factors;
    // Add symbolic factors passed up from children
    symbolicFactors += myData.childSymbolicFactors;

    Ordering keyAsOrdering;
    keyAsOrdering.push_back(ETreeNode->key);
    SymbolicConditional::shared_ptr myConditional;
    SymbolicFactor::shared_ptr mySeparatorFactor;
    boost::tie(myConditional, mySeparatorFactor) = internal::EliminateSymbolic(
        symbolicFactors, keyAsOrdering);

    // Store symbolic elimination results in the parent
    myData.parentData->childSymbolicConditionals.push_back(myConditional);
    myData.parentData->childSymbolicFactors.push_back(mySeparatorFactor);

    sharedNode node = myData.myJTNode;
    const FastVector<SymbolicConditional::shared_ptr>& childConditionals =
        myData.childSymbolicConditionals;

    // Merge our children if they are in our clique - if our conditional has
    // exactly one fewer parent than our child's conditional.
    size_t myNrFrontals = 1;
    const size_t myNrParents = myConditional->nrParents();
    assert(node->newChildren.size() == childConditionals.size());

    gttic(merge_children);
    // First count how many keys, factors and children we'll end up with
    size_t nrKeys = node->orderedFrontalKeys.size();
    size_t nrFactors = node->factors.size();
    size_t nrChildren = 0;
    // Loop over children
    for (size_t i = 0; i < childConditionals.size(); ++i) {
      // Check if we should merge the i^th child
      if (myNrParents + myNrFrontals == childConditionals[i]->nrParents()) {
        // Get a reference to the i, adjusting the index to account for children
        // previously merged and removed from the i list.
        sharedNode child = node->children[i];
        nrKeys += child->orderedFrontalKeys.size();
        nrFactors += child->factors.size();
        nrChildren += child->children.size();
        // Increment number of frontal variables
        myNrFrontals += child->orderedFrontalKeys.size();
      } else {
        nrChildren += 1; // we keep the child
      }
    }

    // now reserve space, and really merge
    node->orderedFrontalKeys.reserve(nrKeys);
    node->factors.reserve(nrFactors);
    typename Node::Children newChildren;
    newChildren.reserve(nrChildren);
    myNrFrontals = 1;
    int combinedProblemSize = (int) (myConditional->size() * symbolicFactors.size());
    // Loop over newChildren
    for (size_t i = 0; i < childConditionals.size(); ++i) {
      // Check if we should merge the i^th child
      sharedNode child = node->children[i];
      if (myNrParents + myNrFrontals == childConditionals[i]->nrParents()) {
        // Get a reference to the i, adjusting the index to account for newChildren
        // previously merged and removed from the i list.
        // Merge keys. For efficiency, we add keys in reverse order at end, calling reverse after..
        node->orderedFrontalKeys.insert(node->orderedFrontalKeys.end(),
            child->orderedFrontalKeys.rbegin(),
            child->orderedFrontalKeys.rend());
        // Merge keys, factors, and children.
        node->factors.insert(node->factors.end(), child->factors.begin(), child->factors.end());
        newChildren.insert(newChildren.end(), child->children.begin(), child->children.end());
        // Increment problem size
        combinedProblemSize = std::max(combinedProblemSize, child->problemSize_);
        // Increment number of frontal variables
        myNrFrontals += child->orderedFrontalKeys.size();
      } else {
        newChildren.push_back(child); // we keep the child
      }
    }
    node->children = newChildren;
    std::reverse(node->orderedFrontalKeys.begin(), node->orderedFrontalKeys.end());
    gttoc(merge_children);
    node->problemSize_ = combinedProblemSize;
  }
}
;

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
  rootData.myJTNode = boost::make_shared<typename Base::Node>(); // Make a dummy node to gather
                                                                 // the junction tree roots
  treeTraversal::DepthFirstForest(eliminationTree, rootData,
      Data::ConstructorTraversalVisitorPre,
      Data::ConstructorTraversalVisitorPostAlg2);

  // Assign roots from the dummy node
  Base::roots_ = rootData.myJTNode->children;

  // Transfer remaining factors from elimination tree
  Base::remainingFactors_ = eliminationTree.remainingFactors();
}

} // namespace gtsam
