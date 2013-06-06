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
#include <boost/bind.hpp>

namespace gtsam {
  
  namespace {
    /* ************************************************************************* */
    template<class BAYESTREE, class GRAPH>
    struct ConstructorTraversalData {
      ConstructorTraversalData* const parentData;
      typename JunctionTreeUnordered<BAYESTREE,GRAPH>::sharedNode myJTNode;
      std::vector<SymbolicConditionalUnordered::shared_ptr> childSymbolicConditionals;
      std::vector<SymbolicFactorUnordered::shared_ptr> childSymbolicFactors;
      ConstructorTraversalData(ConstructorTraversalData* _parentData) : parentData(_parentData) {}
    };

    /* ************************************************************************* */
    // Pre-order visitor function
    template<class BAYESTREE, class GRAPH, class ETREE_NODE>
    ConstructorTraversalData<BAYESTREE,GRAPH> ConstructorTraversalVisitorPre(
      const boost::shared_ptr<ETREE_NODE>& node,
      ConstructorTraversalData<BAYESTREE,GRAPH>& parentData)
    {
      // On the pre-order pass, before children have been visited, we just set up a traversal data
      // structure with its own JT node, and create a child pointer in its parent.
      ConstructorTraversalData<BAYESTREE,GRAPH> myData = ConstructorTraversalData<BAYESTREE,GRAPH>(&parentData);
      myData.myJTNode = boost::make_shared<typename JunctionTreeUnordered<BAYESTREE,GRAPH>::Node>();
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
      // In this post-order visitor, we combine the symbolic elimination results from the
      // elimination tree children and symbolically eliminate the current elimination tree node.  We
      // then check whether each of our elimination tree child nodes should be merged with us.  The
      // check for this is that our number of symbolic elimination parents is exactly 1 less than
      // our child's symbolic elimination parents - this condition indicates that eliminating the
      // current node did not introduce any parents beyond those already in the child.

      // Do symbolic elimination for this node
      std::vector<SymbolicFactorUnordered::shared_ptr> symbolicFactors;
      symbolicFactors.reserve(node->factors.size() + myData.childSymbolicFactors.size());
      // Add symbolic versions of the ETree node factors
      BOOST_FOREACH(const typename GRAPH::sharedFactor& factor, node->factors) {
        symbolicFactors.push_back(boost::make_shared<SymbolicFactorUnordered>(
          SymbolicFactorUnordered::FromKeys(*factor))); }
      // Add symbolic factors passed up from children
      symbolicFactors.insert(symbolicFactors.end(), myData.childSymbolicFactors.begin(), myData.childSymbolicFactors.end());
      std::vector<Key> keyAsVector(1); keyAsVector[0] = node->key;
      std::pair<SymbolicConditionalUnordered::shared_ptr, SymbolicFactorUnordered::shared_ptr> symbolicElimResult =
        EliminateSymbolicUnordered(symbolicFactors, keyAsVector);

      // Store symbolic elimination results in the parent
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
          const typename JunctionTreeUnordered<BAYESTREE,GRAPH>::Node& childToMerge =
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

    /* ************************************************************************* */
    // Elimination traversal data - stores a pointer to the parent data and collects the factors
    // resulting from elimination of the children.  Also sets up BayesTree cliques with parent and
    // child pointers.
    template<class JUNCTIONTREE>
    struct EliminationData {
      EliminationData* const parentData;
      std::vector<typename JUNCTIONTREE::sharedFactor> childFactors;
      boost::shared_ptr<typename JUNCTIONTREE::BayesTreeType::Node> bayesTreeNode;
      EliminationData(EliminationData* _parentData, size_t nChildren) :
        parentData(_parentData),
        bayesTreeNode(boost::make_shared<typename JUNCTIONTREE::BayesTreeType::Node>()) {
          childFactors.reserve(nChildren);
          // Set up BayesTree parent and child pointers
          if(parentData) {
            bayesTreeNode->parent_ = parentData->bayesTreeNode;
            parentData->bayesTreeNode->children.push_back(bayesTreeNode);
          }
      }
    };

    /* ************************************************************************* */
    // Elimination pre-order visitor - just creates the EliminationData structure for the visited
    // node.
    template<class JUNCTIONTREE>
    EliminationData<JUNCTIONTREE> eliminationPreOrderVisitor(
      const typename JUNCTIONTREE::sharedNode& node, EliminationData<JUNCTIONTREE>& parentData)
    {
      return EliminationData<JUNCTIONTREE>(&parentData, node->children.size());
    }

    /* ************************************************************************* */
    // Elimination post-order visitor - combine the child factors with our own factors, add the
    // resulting conditional to the BayesTree, and add the remaining factor to the parent.  The
    // extra argument 'eliminationFunction' is passed from JunctionTree::eliminate using
    // boost::bind.
    template<class JUNCTIONTREE>
    void eliminationPostOrderVisitor(const typename JUNCTIONTREE::sharedNode& node, EliminationData<JUNCTIONTREE>& myData,
      const typename JUNCTIONTREE::Eliminate& eliminationFunction)
    {
      // Typedefs
      typedef typename JUNCTIONTREE::sharedFactor sharedFactor;
      typedef typename JUNCTIONTREE::FactorType FactorType;
      typedef typename JUNCTIONTREE::ConditionalType ConditionalType;
      typedef typename JUNCTIONTREE::BayesTreeType::Node BTNode;

      // Gather factors
      assert(node->children.size() == myData.childFactors.size());
      std::vector<sharedFactor> gatheredFactors;
      gatheredFactors.reserve(node->factors.size() + node->children.size());
      gatheredFactors.insert(gatheredFactors.end(), node->factors.begin(), node->factors.end());
      gatheredFactors.insert(gatheredFactors.end(), myData.childFactors.begin(), myData.childFactors.end());

      // Do dense elimination step
      std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<FactorType> > eliminationResult =
        eliminationFunction(gatheredFactors, node->keys);

      // Store conditional in BayesTree clique
      myData.bayesTreeNode->conditional_ = eliminationResult.first;

      // Store remaining factor in parent's gathered factors
      myData.parentData->childFactors.push_back(eliminationResult.second);
    }
  }

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  template<class ETREE>
  JunctionTreeUnordered<BAYESTREE,GRAPH>
    JunctionTreeUnordered<BAYESTREE,GRAPH>::FromEliminationTree(const ETREE& eliminationTree)
  {
    // Here we rely on the BayesNet having been produced by this elimination tree, such that the
    // conditionals are arranged in DFS post-order.  We traverse the elimination tree, and inspect
    // the symbolic conditional corresponding to each node.  The elimination tree node is added to
    // the same clique with its parent if it has exactly one more Bayes net conditional parent than
    // does its elimination tree parent.

    // Traverse the elimination tree, doing symbolic elimination and merging nodes as we go.  Gather
    // the created junction tree roots in a dummy Node.
    typedef typename ETREE::Node ETreeNode;
    ConstructorTraversalData<BAYESTREE, GRAPH> rootData(0);
    rootData.myJTNode = boost::make_shared<Node>(); // Make a dummy node to gather the junction tree roots
    treeTraversal::DepthFirstForest(eliminationTree, rootData,
      ConstructorTraversalVisitorPre<BAYESTREE,GRAPH,ETreeNode>, ConstructorTraversalVisitorPost<BAYESTREE,GRAPH,ETreeNode>);

    // Assign roots from the dummy node
    This result;
    result.roots_ = rootData.myJTNode->children;

    // Transfer remaining factors from elimination tree
    result.remainingFactors_ = eliminationTree.remainingFactors();

    return result;
  }

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  JunctionTreeUnordered<BAYESTREE,GRAPH>& JunctionTreeUnordered<BAYESTREE,GRAPH>::operator=(const This& other)
  {
    // Start by duplicating the tree.
    roots_ = treeTraversal::CloneForest(other);

    // Assign the remaining factors - these are pointers to factors in the original factor graph and
    // we do not clone them.
    remainingFactors_ = other.remainingFactors_;

    return *this;
  }

  /* ************************************************************************* */
  template<class BAYESTREE, class GRAPH>
  std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<GRAPH> >
    JunctionTreeUnordered<BAYESTREE,GRAPH>::eliminate(const Eliminate& function) const
  {
    // Do elimination (depth-first traversal).  The rootsContainer stores a 'dummy' BayesTree node
    // that contains all of the roots as its children.  rootsContainer also stores the remaining
    // uneliminated factors passed up from the roots.
    EliminationData<This> rootsContainer(0, roots_.size());
    treeTraversal::DepthFirstForest(*this, rootsContainer, eliminationPreOrderVisitor<This>,
      boost::bind(eliminationPostOrderVisitor<This>, _1, _2, function));

    // Create BayesTree from roots stored in the dummy BayesTree node.
    boost::shared_ptr<BayesTreeType> result = boost::make_shared<BayesTreeType>();
    BOOST_FOREACH(const typename BayesTreeType::sharedNode& root, rootsContainer.bayesTreeNode->children)
      result->insertRoot(root);

    // Add remaining factors that were not involved with eliminated variables
    boost::shared_ptr<FactorGraphType> allRemainingFactors = boost::make_shared<FactorGraphType>();
    allRemainingFactors->push_back(remainingFactors_.begin(), remainingFactors_.end());
    allRemainingFactors->push_back(rootsContainer.childFactors.begin(), rootsContainer.childFactors.end());

    // Return result
    return std::make_pair(result, allRemainingFactors);
  }

} //namespace gtsam
