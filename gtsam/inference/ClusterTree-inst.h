/**
 * @file ClusterTree-inst.h
 * @date Oct 8, 2013
 * @author Kai Ni
 * @author Richard Roberts
 * @author Frank Dellaert
 * @brief Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#include <gtsam/inference/ClusterTree.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/treeTraversal-inst.h>

#include <boost/foreach.hpp>
#include <boost/bind.hpp>

namespace gtsam {
namespace {
/* ************************************************************************* */
// Elimination traversal data - stores a pointer to the parent data and collects the factors
// resulting from elimination of the children.  Also sets up BayesTree cliques with parent and
// child pointers.
template<class CLUSTERTREE>
struct EliminationData {
  EliminationData* const parentData;
  size_t myIndexInParent;
  FastVector<typename CLUSTERTREE::sharedFactor> childFactors;
  boost::shared_ptr<typename CLUSTERTREE::BayesTreeType::Node> bayesTreeNode;
  EliminationData(EliminationData* _parentData, size_t nChildren) :
      parentData(_parentData), bayesTreeNode(
          boost::make_shared<typename CLUSTERTREE::BayesTreeType::Node>()) {
    if (parentData) {
      myIndexInParent = parentData->childFactors.size();
      parentData->childFactors.push_back(typename CLUSTERTREE::sharedFactor());
    } else {
      myIndexInParent = 0;
    }
    // Set up BayesTree parent and child pointers
    if (parentData) {
      if (parentData->parentData) // If our parent is not the dummy node
        bayesTreeNode->parent_ = parentData->bayesTreeNode;
      parentData->bayesTreeNode->children.push_back(bayesTreeNode);
    }
  }
};

/* ************************************************************************* */
// Elimination pre-order visitor - just creates the EliminationData structure for the visited
// node.
template<class CLUSTERTREE>
EliminationData<CLUSTERTREE> eliminationPreOrderVisitor(
    const typename CLUSTERTREE::sharedNode& node,
    EliminationData<CLUSTERTREE>& parentData) {
  EliminationData<CLUSTERTREE> myData(&parentData, node->children.size());
  myData.bayesTreeNode->problemSize_ = node->problemSize();
  return myData;
}

/* ************************************************************************* */
// Elimination post-order visitor - combine the child factors with our own factors, add the
// resulting conditional to the BayesTree, and add the remaining factor to the parent.
template<class CLUSTERTREE>
struct EliminationPostOrderVisitor {
  const typename CLUSTERTREE::Eliminate& eliminationFunction;
  typename CLUSTERTREE::BayesTreeType::Nodes& nodesIndex;
  EliminationPostOrderVisitor(
      const typename CLUSTERTREE::Eliminate& eliminationFunction,
      typename CLUSTERTREE::BayesTreeType::Nodes& nodesIndex) :
      eliminationFunction(eliminationFunction), nodesIndex(nodesIndex) {
  }
  void operator()(const typename CLUSTERTREE::sharedNode& node,
      EliminationData<CLUSTERTREE>& myData) {
    // Typedefs
    typedef typename CLUSTERTREE::sharedFactor sharedFactor;
    typedef typename CLUSTERTREE::FactorType FactorType;
    typedef typename CLUSTERTREE::FactorGraphType FactorGraphType;
    typedef typename CLUSTERTREE::ConditionalType ConditionalType;
    typedef typename CLUSTERTREE::BayesTreeType::Node BTNode;

    // Gather factors
    FactorGraphType gatheredFactors;
    gatheredFactors.reserve(node->factors.size() + node->children.size());
    gatheredFactors += node->factors;
    gatheredFactors += myData.childFactors;

    // Check for Bayes tree orphan subtrees, and add them to our children
    BOOST_FOREACH(const sharedFactor& f, node->factors) {
      if (const BayesTreeOrphanWrapper<BTNode>* asSubtree =
          dynamic_cast<const BayesTreeOrphanWrapper<BTNode>*>(f.get())) {
        myData.bayesTreeNode->children.push_back(asSubtree->clique);
        asSubtree->clique->parent_ = myData.bayesTreeNode;
      }
    }

    // Do dense elimination step
    std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<FactorType> > eliminationResult =
        eliminationFunction(gatheredFactors, node->orderedFrontalKeys);

    // Store conditional in BayesTree clique, and in the case of ISAM2Clique also store the remaining factor
    myData.bayesTreeNode->setEliminationResult(eliminationResult);

    // Fill nodes index - we do this here instead of calling insertRoot at the end to avoid
    // putting orphan subtrees in the index - they'll already be in the index of the ISAM2
    // object they're added to.
    BOOST_FOREACH(const Key& j, myData.bayesTreeNode->conditional()->frontals())
      nodesIndex.insert(std::make_pair(j, myData.bayesTreeNode));

    // Store remaining factor in parent's gathered factors
    if (!eliminationResult.second->empty())
      myData.parentData->childFactors[myData.myIndexInParent] =
          eliminationResult.second;
  }
};
}

/* ************************************************************************* */
template<class BAYESTREE, class GRAPH>
void ClusterTree<BAYESTREE, GRAPH>::Cluster::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << " (" << problemSize_ << ")";
  PrintKeyVector(orderedFrontalKeys);
}

/* ************************************************************************* */
template<class BAYESTREE, class GRAPH>
void ClusterTree<BAYESTREE, GRAPH>::Cluster::mergeChildren(
    const std::vector<bool>& merge) {
  gttic(Cluster::mergeChildren);
  size_t nrChildren = children.size();

  // Count how many keys, factors and children we'll end up with
  size_t nrKeys = orderedFrontalKeys.size();
  size_t nrFactors = factors.size();
  size_t nrNewChildren = 0;
  // Loop over children
  for (size_t i = 0; i < nrChildren; ++i) {
    if (merge[i]) {
      // Get a reference to the i, adjusting the index to account for children
      // previously merged and removed from the i list.
      sharedNode child = children[i];
      nrKeys += child->orderedFrontalKeys.size();
      nrFactors += child->factors.size();
      nrNewChildren += child->children.size();
    } else {
      nrNewChildren += 1; // we keep the child
    }
  }

  // now reserve space, and really merge
  orderedFrontalKeys.reserve(nrKeys);
  factors.reserve(nrFactors);
  typename Node::Children newChildren;
  newChildren.reserve(nrNewChildren);
  // Loop over newChildren
  for (size_t i = 0; i < nrChildren; ++i) {
    // Check if we should merge the i^th child
    sharedNode child = children[i];
    if (merge[i]) {
      // Get a reference to the i, adjusting the index to account for newChildren
      // previously merged and removed from the i list.
      // Merge keys. For efficiency, we add keys in reverse order at end, calling reverse after..
      orderedFrontalKeys.insert(orderedFrontalKeys.end(),
          child->orderedFrontalKeys.rbegin(), child->orderedFrontalKeys.rend());
      // Merge keys, factors, and children.
      factors.insert(factors.end(), child->factors.begin(),
          child->factors.end());
      newChildren.insert(newChildren.end(), child->children.begin(),
          child->children.end());
      // Increment problem size
      problemSize_ = std::max(problemSize_, child->problemSize_);
      // Increment number of frontal variables
    } else {
      newChildren.push_back(child); // we keep the child
    }
  }
  children = newChildren;
  std::reverse(orderedFrontalKeys.begin(), orderedFrontalKeys.end());
}

/* ************************************************************************* */
template<class BAYESTREE, class GRAPH>
void ClusterTree<BAYESTREE, GRAPH>::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  treeTraversal::PrintForest(*this, s, keyFormatter);
}

/* ************************************************************************* */
template<class BAYESTREE, class GRAPH>
ClusterTree<BAYESTREE, GRAPH>& ClusterTree<BAYESTREE, GRAPH>::operator=(
    const This& other) {
  // Start by duplicating the tree.
  roots_ = treeTraversal::CloneForest(other);

  // Assign the remaining factors - these are pointers to factors in the original factor graph and
  // we do not clone them.
  remainingFactors_ = other.remainingFactors_;

  return *this;
}

/* ************************************************************************* */
template<class BAYESTREE, class GRAPH>
std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<GRAPH> > ClusterTree<
    BAYESTREE, GRAPH>::eliminate(const Eliminate& function) const {
  gttic(ClusterTree_eliminate);
  // Do elimination (depth-first traversal).  The rootsContainer stores a 'dummy' BayesTree node
  // that contains all of the roots as its children.  rootsContainer also stores the remaining
  // uneliminated factors passed up from the roots.
  boost::shared_ptr<BayesTreeType> result = boost::make_shared<BayesTreeType>();
  EliminationData<This> rootsContainer(0, roots_.size());
  EliminationPostOrderVisitor<This> visitorPost(function, result->nodes_);
  {
    TbbOpenMPMixedScope threadLimiter; // Limits OpenMP threads since we're mixing TBB and OpenMP
    treeTraversal::DepthFirstForestParallel(*this, rootsContainer,
        eliminationPreOrderVisitor<This>, visitorPost, 10);
  }

  // Create BayesTree from roots stored in the dummy BayesTree node.
  result->roots_.insert(result->roots_.end(),
      rootsContainer.bayesTreeNode->children.begin(),
      rootsContainer.bayesTreeNode->children.end());

  // Add remaining factors that were not involved with eliminated variables
  boost::shared_ptr<FactorGraphType> allRemainingFactors = boost::make_shared<
      FactorGraphType>();
  allRemainingFactors->reserve(
      remainingFactors_.size() + rootsContainer.childFactors.size());
  allRemainingFactors->push_back(remainingFactors_.begin(),
      remainingFactors_.end());
  BOOST_FOREACH(const sharedFactor& factor, rootsContainer.childFactors)
    if (factor)
      allRemainingFactors->push_back(factor);

  // Return result
  return std::make_pair(result, allRemainingFactors);
}

}
