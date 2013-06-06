/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file JunctionTree-inl.h
 * @date Feb 4, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @author Richard Roberts
 * @brief The junction tree, template bodies
 */

#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/JunctionTreeUnordered.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>

#include <boost/foreach.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  typename JunctionTreeUnordered<BAYESTREE,GRAPH>::sharedFactor
    JunctionTreeUnordered<BAYESTREE,GRAPH>::Node::eliminate(
    const boost::shared_ptr<BayesTreeType>& output,
    const Eliminate& function, const std::vector<sharedFactor>& childrenResults) const
  {
    // This function eliminates one node (Node::eliminate) - see below eliminate for the whole tree.

    assert(childrenResults.size() == children.size());

    // Gather factors
    std::vector<sharedFactor> gatheredFactors;
    gatheredFactors.reserve(factors.size() + children.size());
    gatheredFactors.insert(gatheredFactors.end(), factors.begin(), factors.end());
    gatheredFactors.insert(gatheredFactors.end(), childrenResults.begin(), childrenResults.end());

    // Do dense elimination step
    std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<FactorType> > eliminationResult =
      function(gatheredFactors, keys);

    // Add conditional to BayesNet
    output->push_back(eliminationResult.first);

    // Return result
    return eliminationResult.second;
  }


  namespace {
    /* ************************************************************************* */
    template<class BAYESTREE, class GRAPH>
    struct ConstructorTraversalData {
      const ConstructorTraversalData* parentData;
      typename JunctionTreeUnordered<BAYESTREE,GRAPH>::sharedNode myJTNode;
      std::vector<SymbolicConditionalUnordered::shared_ptr> childSymbolicConditionals;
      std::vector<SymbolicFactorUnordered::shared_ptr> childSymbolicFactors;
      ConstructorTraversalData(const ConstructorTraversalData* _parentData) : parentData(_parentData) {}
    };

    /* ************************************************************************* */
    // Pre-order visitor function
    template<class BAYESTREE, class GRAPH, class ETREE_NODE>
    ConstructorTraversalData<BAYESTREE,GRAPH> ConstructorTraversalVisitorPre(
      const boost::shared_ptr<ETREE_NODE>& node,
      const ConstructorTraversalData<BAYESTREE,GRAPH>& parentData)
    {
      // On the pre-order pass, before children have been visited, we just set up a traversal data
      // structure with its own JT node, and create a child pointer in its parent.
      ConstructorTraversalData<BAYESTREE,GRAPH> myData = ConstructorTraversalData<BAYESTREE,GRAPH>(&parentData);
      myData.myJTNode = boost::make_shared<typename JunctionTreeUnordered<BAYESREE,GRAPH>::Node>();
      myData.myJTNode->keys.push_back(node->key);
      myData.myJTNode->factors.insert(myData.myJTNode->factors.begin(), node->factors.begin(), node->factors.end());
      parentData.myJTNode->children.push_back(myData.myJTNode);
      return myData;
    }

    /* ************************************************************************* */
    // Post-order visitor function
    template<class BAYESTREE, class GRAPH, class ETREE_NODE>
    void ConstructorTraversalVisitorPost(
      const boost::shared_ptr<ETREE_NODE>& node,
      const ConstructorTraversalData<BAYESTREE,GRAPH>& myData)
    {
      // Do symbolic elimination for this node
      std::vector<SymbolicFactorUnordered::shared_ptr> symbolicFactors;
      symbolicFactors.reserve(node->factors.size() + myData.childSymbolicFactors.size());
      BOOST_FOREACH(const typename GRAPH::sharedFactor& factor, node->factors) {
        symbolicFactors.push_back(boost::make_shared<SymbolicFactorUnordered>(factor)); }
      symbolicFactors.insert(symbolicFactors.end(), myData.childSymbolicFactors.begin(), myData.childSymbolicFactors.end());
      std::vector<Key> keyAsVector(1); keyAsVector[0] = node->key;
      std::pair<SymbolicConditionalUnordered::shared_ptr, SymbolicFactorUnordered::shared_ptr> symbolicElimResult =
        EliminateSymbolicUnordered(symbolicFactors, keyAsVector);

      // Store symbolic elimination results
      myData.parentData->childSymbolicConditionals.push_back(symbolicElimResult.first);
      myData.parentData->childSymbolicFactors.push_back(symbolicElimResult.second);

      // Merge our children if they are in our clique - if our conditional has exactly one fewer
      // parent than our child's conditional.
      const size_t myNrParents = symbolicElimResult.first->nrParents();
      size_t nrMergedChildren = 0;
      assert(myData.myJTNode->children.size() == myData.childSymbolicConditionals.size());
      // Loop over children
      for(size_t child = 0; child < myData.childSymbolicConditionals.size(); ++child) {
        // Check if we should merge the child
        if(myNrParents + 1 == myData.childSymbolicConditionals[child]->nrParents()) {
          // Get a reference to the child, adjusting the index to account for children previously
          // merged and removed from the child list.
          const typename JunctionTreeUnordered<BAYESREE,GRAPH>::Node& childToMerge =
            *myData.myJTNode->children[child - nrMergedChildren];
          // Merge keys, factors, and children.
          myData.myJTNode->keys.insert(myData.myJTNode->keys.end(), childToMerge.keys.begin(), childToMerge.keys.end());
          myData.myJTNode->factors.insert(myData.myJTNode->factors.end(), childToMerge.factors.begin(), childToMerge.factors.end());
          myData.myJTNode->children.insert(myData.myJTNode->children.end(), childToMerge.children.begin(), childToMerge.children.end());
          // Remove child from list.
          myData.myJTNode->children.erase(myData.myJTNode->children.begin() + child - nrMergedChildren);
        }
      }
    }
  }

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  template<class ETREE>
  JunctionTreeUnordered<BAYESTREE,GRAPH>::JunctionTreeUnordered(const ETREE& eliminationTree)
  {
    // Here we rely on the BayesNet having been produced by this elimination tree, such that the
    // conditionals are arranged in DFS post-order.  We traverse the elimination tree, and inspect
    // the symbolic conditional corresponding to each node.  The elimination tree node is added to
    // the same clique with its parent if it has exactly one more Bayes net conditional parent than
    // does its elimination tree parent.

    // Traverse the elimination tree, doing symbolic elimination and merging nodes as we go.  Gather
    // the created junction tree roots in a dummy Node.
    ConstructorTraversalData<BAYESTREE, GRAPH> rootData(0);
    rootData.myJTNode = boost::make_shared<Node>(); // Make a dummy node to gather the junction tree roots
    treeTraversal.DepthFirstForest(eliminationTree, rootData,
      ConstructorTraversalVisitorPre<BAYESREE,GRAPH>, ConstructorTraversalVisitorPost<BAYESTREE,GRAPH>);

    // Assign roots from the dummy node
    roots_ = rootData.myJTNode->children;

    // Transfer remaining factors from elimination tree
    remainingFactors_ = eliminationTree.remainingFactors();
  }

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  JunctionTreeUnordered<BAYESTREE,GRAPH>::JunctionTreeUnordered(const This& other)
  {

  }

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  JunctionTreeUnordered<BAYESTREE,GRAPH>& JunctionTreeUnordered<BAYESREE,GRAPH>::operator=(const This& other)
  {

  }

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<GRAPH> >
    JunctionTreeUnordered<BAYESTREE,GRAPH>::eliminate(const Eliminate& function) const
  {
    // Allocate result
    boost::shared_ptr<BayesTreeType> result = boost::make_shared<BayesTreeType>();

    // Run tree elimination algorithm
    std::vector<sharedFactor> remainingFactors = inference::EliminateTree(result, *this, function);

    // Add remaining factors that were not involved with eliminated variables
    boost::shared_ptr<FactorGraphType> allRemainingFactors = boost::make_shared<FactorGraphType>();
    allRemainingFactors->push_back(remainingFactors_.begin(), remainingFactors_.end());
    allRemainingFactors->push_back(remainingFactors.begin(), remainingFactors.end());

    // Return result
    return std::make_pair(result, allRemainingFactors);
  }

} //namespace gtsam
