/**
 * @file ClusterTree-inst.h
 * @date Oct 8, 2013
 * @author Kai Ni
 * @author Richard Roberts
 * @author Frank Dellaert
 * @brief Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <gtsam/inference/ClusterTree.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/treeTraversal-inst.h>

#ifdef GTSAM_USE_TBB
#include <mutex>
#endif

namespace gtsam {

/* ************************************************************************* */
template<class GRAPH>
void ClusterTree<GRAPH>::Cluster::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << " (" << problemSize_ << ")";
  PrintKeyVector(orderedFrontalKeys);
}

/* ************************************************************************* */
template <class GRAPH>
std::vector<size_t> ClusterTree<GRAPH>::Cluster::nrFrontalsOfChildren() const {
  std::vector<size_t> nrFrontals;
  nrFrontals.reserve(nrChildren());
  for (const sharedNode& child : children)
    nrFrontals.push_back(child->nrFrontals());
  return nrFrontals;
}

/* ************************************************************************* */
template <class GRAPH>
void ClusterTree<GRAPH>::Cluster::merge(const std::shared_ptr<Cluster>& cluster) {
  // Merge keys. For efficiency, we add keys in reverse order at end, calling reverse after..
  orderedFrontalKeys.insert(orderedFrontalKeys.end(), cluster->orderedFrontalKeys.rbegin(),
                            cluster->orderedFrontalKeys.rend());
  factors.push_back(cluster->factors);
  children.insert(children.end(), cluster->children.begin(), cluster->children.end());
  // Increment problem size
  problemSize_ = std::max(problemSize_, cluster->problemSize_);
}

/* ************************************************************************* */
template<class GRAPH>
void ClusterTree<GRAPH>::Cluster::mergeChildren(
    const std::vector<bool>& merge) {
  gttic(Cluster_mergeChildren);
  assert(merge.size() == this->children.size());

  // Count how many keys, factors and children we'll end up with
  size_t nrKeys = orderedFrontalKeys.size();
  size_t nrFactors = factors.size();
  size_t nrNewChildren = 0;
  // Loop over children
  size_t i = 0;
  for(const sharedNode& child: this->children) {
    if (merge[i]) {
      nrKeys += child->orderedFrontalKeys.size();
      nrFactors += child->factors.size();
      nrNewChildren += child->nrChildren();
    } else {
      nrNewChildren += 1; // we keep the child
    }
    ++i;
  }

  // now reserve space, and really merge
  auto oldChildren = this->children;
  this->children.clear();
  this->children.reserve(nrNewChildren);
  orderedFrontalKeys.reserve(nrKeys);
  factors.reserve(nrFactors);
  i = 0;
  for (const sharedNode& child : oldChildren) {
    if (merge[i]) {
      this->merge(child);
    } else {
      this->addChild(child);  // we keep the child
    }
    ++i;
  }
  std::reverse(orderedFrontalKeys.begin(), orderedFrontalKeys.end());
}

/* ************************************************************************* */
template <class GRAPH>
void ClusterTree<GRAPH>::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  treeTraversal::PrintForest(*this, s, keyFormatter);
}

/* ************************************************************************* */
template <class GRAPH>
ClusterTree<GRAPH>& ClusterTree<GRAPH>::operator=(const This& other) {
  // Start by duplicating the tree.
  roots_ = treeTraversal::CloneForest(other);
  return *this;
}

/* ************************************************************************* */
// Elimination traversal data - stores a pointer to the parent data and collects
// the factors resulting from elimination of the children.  Also sets up BayesTree
// cliques with parent and child pointers.
template<class CLUSTERTREE>
struct EliminationData {
  // Typedefs
  typedef typename CLUSTERTREE::sharedFactor sharedFactor;
  typedef typename CLUSTERTREE::FactorType FactorType;
  typedef typename CLUSTERTREE::FactorGraphType FactorGraphType;
  typedef typename CLUSTERTREE::ConditionalType ConditionalType;
  typedef typename CLUSTERTREE::BayesTreeType::Node BTNode;

  EliminationData* const parentData;
  size_t myIndexInParent;
  FastVector<sharedFactor> childFactors;
  std::shared_ptr<BTNode> bayesTreeNode;
#ifdef GTSAM_USE_TBB
  std::shared_ptr<std::mutex> writeLock;
#endif

  EliminationData(EliminationData* _parentData, size_t nChildren) :
      parentData(_parentData), bayesTreeNode(std::make_shared<BTNode>())
#ifdef GTSAM_USE_TBB
      , writeLock(std::make_shared<std::mutex>())
#endif
    {
    if (parentData) {
#ifdef GTSAM_USE_TBB
      parentData->writeLock->lock();
#endif
      myIndexInParent = parentData->childFactors.size();
      parentData->childFactors.push_back(sharedFactor());
#ifdef GTSAM_USE_TBB
      parentData->writeLock->unlock();
#endif
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

  // Elimination pre-order visitor - creates the EliminationData structure for the visited node.
  static EliminationData EliminationPreOrderVisitor(
      const typename CLUSTERTREE::sharedNode& node,
      EliminationData& parentData) {
    assert(node);
    EliminationData myData(&parentData, node->nrChildren());
    myData.bayesTreeNode->problemSize_ = node->problemSize();
    return myData;
  }

  // Elimination post-order visitor - combine the child factors with our own factors, add the
  // resulting conditional to the BayesTree, and add the remaining factor to the parent.
  class EliminationPostOrderVisitor {
    const typename CLUSTERTREE::Eliminate& eliminationFunction_;
    typename CLUSTERTREE::BayesTreeType::Nodes& nodesIndex_;

  public:
    // Construct functor
    EliminationPostOrderVisitor(
        const typename CLUSTERTREE::Eliminate& eliminationFunction,
        typename CLUSTERTREE::BayesTreeType::Nodes& nodesIndex) :
        eliminationFunction_(eliminationFunction), nodesIndex_(nodesIndex) {
    }

    // Function that does the HEAVY lifting
    void operator()(const typename CLUSTERTREE::sharedNode& node, EliminationData& myData) {
      assert(node);

      // Gather factors
      FactorGraphType gatheredFactors;
      gatheredFactors.reserve(node->factors.size() + node->nrChildren());
      gatheredFactors += node->factors;
      gatheredFactors += myData.childFactors;

      // Check for Bayes tree orphan subtrees, and add them to our children
      // TODO(frank): should this really happen here?
      for (const sharedFactor& factor: node->factors) {
        auto asSubtree = dynamic_cast<const BayesTreeOrphanWrapper<BTNode>*>(factor.get());
        if (asSubtree) {
          myData.bayesTreeNode->children.push_back(asSubtree->clique);
          asSubtree->clique->parent_ = myData.bayesTreeNode;
        }
      }

      // >>>>>>>>>>>>>> Do dense elimination step >>>>>>>>>>>>>>>>>>>>>>>>>>>>>
      auto eliminationResult = eliminationFunction_(gatheredFactors, node->orderedFrontalKeys);
      // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

      // Store conditional in BayesTree clique, and in the case of ISAM2Clique also store the
      // remaining factor
      myData.bayesTreeNode->setEliminationResult(eliminationResult);

      // Fill nodes index - we do this here instead of calling insertRoot at the end to avoid
      // putting orphan subtrees in the index - they'll already be in the index of the ISAM2
      // object they're added to.
      for (const Key& j: myData.bayesTreeNode->conditional()->frontals())
        nodesIndex_.insert(std::make_pair(j, myData.bayesTreeNode));

      // Store remaining factor in parent's gathered factors
      if (!eliminationResult.second->empty()) {
#ifdef GTSAM_USE_TBB
        myData.parentData->writeLock->lock();
#endif
        myData.parentData->childFactors[myData.myIndexInParent] = eliminationResult.second;
#ifdef GTSAM_USE_TBB
        myData.parentData->writeLock->unlock();
#endif
      }
    }
  };
};

/* ************************************************************************* */
template<class BAYESTREE, class GRAPH>
EliminatableClusterTree<BAYESTREE, GRAPH>& EliminatableClusterTree<BAYESTREE, GRAPH>::operator=(
    const This& other) {
  ClusterTree<GRAPH>::operator=(other);

  // Assign the remaining factors - these are pointers to factors in the original factor graph and
  // we do not clone them.
  remainingFactors_ = other.remainingFactors_;

  return *this;
}

/* ************************************************************************* */
template <class BAYESTREE, class GRAPH>
std::pair<std::shared_ptr<BAYESTREE>, std::shared_ptr<GRAPH> >
EliminatableClusterTree<BAYESTREE, GRAPH>::eliminate(const Eliminate& function) const {
  gttic(ClusterTree_eliminate);
  // Do elimination (depth-first traversal).  The rootsContainer stores a 'dummy' BayesTree node
  // that contains all of the roots as its children.  rootsContainer also stores the remaining
  // un-eliminated factors passed up from the roots.
  std::shared_ptr<BayesTreeType> result = std::make_shared<BayesTreeType>();

  typedef EliminationData<This> Data;
  Data rootsContainer(0, this->nrRoots());

  typename Data::EliminationPostOrderVisitor visitorPost(function, result->nodes_);
  {
    TbbOpenMPMixedScope threadLimiter;  // Limits OpenMP threads since we're mixing TBB and OpenMP
    treeTraversal::DepthFirstForestParallel(*this, rootsContainer, Data::EliminationPreOrderVisitor,
                                            visitorPost, 10);
  }

  // Create BayesTree from roots stored in the dummy BayesTree node.
  result->roots_.insert(result->roots_.end(), rootsContainer.bayesTreeNode->children.begin(),
                        rootsContainer.bayesTreeNode->children.end());

  // Add remaining factors that were not involved with eliminated variables
  std::shared_ptr<FactorGraphType> remaining = std::make_shared<FactorGraphType>();
  remaining->reserve(remainingFactors_.size() + rootsContainer.childFactors.size());
  remaining->push_back(remainingFactors_.begin(), remainingFactors_.end());
  for (const sharedFactor& factor : rootsContainer.childFactors) {
    if (factor)
      remaining->push_back(factor);
  }

  // Return result
  return std::make_pair(result, remaining);
}

} // namespace gtsam
