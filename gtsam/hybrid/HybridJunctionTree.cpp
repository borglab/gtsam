/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridJunctionTree.cpp
 * @date Mar 11, 2022
 * @author Fan Jiang
 */

#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/inference/JunctionTree-inst.h>
#include <gtsam/inference/Key.h>

#include <unordered_map>

namespace gtsam {

// Instantiate base classes
template class EliminatableClusterTree<HybridBayesTree,
                                       HybridGaussianFactorGraph>;
template class JunctionTree<HybridBayesTree, HybridGaussianFactorGraph>;

struct HybridConstructorTraversalData {
  typedef HybridJunctionTree::Node Node;
  typedef
      typename JunctionTree<HybridBayesTree,
                            HybridGaussianFactorGraph>::sharedNode sharedNode;

  HybridConstructorTraversalData* const parentData;
  sharedNode junctionTreeNode;
  FastVector<SymbolicConditional::shared_ptr> childSymbolicConditionals;
  FastVector<SymbolicFactor::shared_ptr> childSymbolicFactors;
  KeySet discreteKeys;

  // Small inner class to store symbolic factors
  class SymbolicFactors : public FactorGraph<Factor> {};

  HybridConstructorTraversalData(HybridConstructorTraversalData* _parentData)
      : parentData(_parentData) {}

  // Pre-order visitor function
  static HybridConstructorTraversalData ConstructorTraversalVisitorPre(
      const boost::shared_ptr<HybridEliminationTree::Node>& node,
      HybridConstructorTraversalData& parentData) {
    // On the pre-order pass, before children have been visited, we just set up
    // a traversal data structure with its own JT node, and create a child
    // pointer in its parent.
    HybridConstructorTraversalData data =
        HybridConstructorTraversalData(&parentData);
    data.junctionTreeNode = boost::make_shared<Node>(node->key, node->factors);
    parentData.junctionTreeNode->addChild(data.junctionTreeNode);

    // Add all the discrete keys in the hybrid factors to the current data
    for (HybridFactor::shared_ptr& f : node->factors) {
      for (auto& k : f->discreteKeys()) {
        data.discreteKeys.insert(k.first);
      }
    }

    return data;
  }

  // Post-order visitor function
  static void ConstructorTraversalVisitorPost(
      const boost::shared_ptr<HybridEliminationTree::Node>& node,
      const HybridConstructorTraversalData& data) {
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
    symbolicFactors.reserve(node->factors.size() +
                            data.childSymbolicFactors.size());
    // Add ETree node factors
    symbolicFactors += node->factors;
    // Add symbolic factors passed up from children
    symbolicFactors += data.childSymbolicFactors;

    Ordering keyAsOrdering;
    keyAsOrdering.push_back(node->key);
    SymbolicConditional::shared_ptr conditional;
    SymbolicFactor::shared_ptr separatorFactor;
    boost::tie(conditional, separatorFactor) =
        internal::EliminateSymbolic(symbolicFactors, keyAsOrdering);

    // Store symbolic elimination results in the parent
    data.parentData->childSymbolicConditionals.push_back(conditional);
    data.parentData->childSymbolicFactors.push_back(separatorFactor);
    data.parentData->discreteKeys.merge(data.discreteKeys);

    sharedNode jt_node = data.junctionTreeNode;
    const FastVector<SymbolicConditional::shared_ptr>& childConditionals =
        data.childSymbolicConditionals;
    jt_node->problemSize_ = (int)(conditional->size() * symbolicFactors.size());

    // Merge our children if they are in our clique - if our conditional has
    // exactly one fewer parent than our child's conditional.
    const size_t nrParents = conditional->nrParents();
    const size_t nrChildren = jt_node->nrChildren();
    assert(childConditionals.size() == nrChildren);

    // decide which children to merge, as index into children
    std::vector<size_t> nrChildrenFrontals = jt_node->nrFrontalsOfChildren();
    std::vector<bool> merge(nrChildren, false);
    size_t nrFrontals = 1;
    for (size_t i = 0; i < nrChildren; i++) {
      // Check if we should merge the i^th child
      if (nrParents + nrFrontals == childConditionals[i]->nrParents()) {
        const bool myType =
            data.discreteKeys.exists(conditional->frontals()[0]);
        const bool theirType =
            data.discreteKeys.exists(childConditionals[i]->frontals()[0]);

        if (myType == theirType) {
          // Increment number of frontal variables
          nrFrontals += nrChildrenFrontals[i];
          merge[i] = true;
        }
      }
    }

    // now really merge
    jt_node->mergeChildren(merge);
  }
};

/* ************************************************************************* */
HybridJunctionTree::HybridJunctionTree(
    const HybridEliminationTree& eliminationTree)
    : etree_(eliminationTree) {
  gttic(JunctionTree_FromEliminationTree);
  // Here we rely on the BayesNet having been produced by this elimination tree,
  // such that the conditionals are arranged in DFS post-order.  We traverse the
  // elimination tree, and inspect the symbolic conditional corresponding to
  // each node.  The elimination tree node is added to the same clique with its
  // parent if it has exactly one more Bayes net conditional parent than
  // does its elimination tree parent.

  // Traverse the elimination tree, doing symbolic elimination and merging nodes
  // as we go.  Gather the created junction tree roots in a dummy Node.
  typedef HybridConstructorTraversalData Data;
  Data rootData(0);
  rootData.junctionTreeNode =
      boost::make_shared<typename Base::Node>();  // Make a dummy node to gather
                                                  // the junction tree roots
  treeTraversal::DepthFirstForest(eliminationTree, rootData,
                                  Data::ConstructorTraversalVisitorPre,
                                  Data::ConstructorTraversalVisitorPost);

  // Assign roots from the dummy node
  this->addChildrenAsRoots(rootData.junctionTreeNode);

  // Transfer remaining factors from elimination tree
  Base::remainingFactors_ = eliminationTree.remainingFactors();
}

/* ************************************************************************* */
std::pair<boost::shared_ptr<HybridBayesTree>,
          boost::shared_ptr<HybridGaussianFactorGraph>>
HybridJunctionTree::eliminateContinuous(
    const Eliminate& function, const HybridGaussianFactorGraph& graph,
    const Ordering& continuous_ordering) const {
  HybridBayesTree::shared_ptr continuousBayesTree;
  HybridGaussianFactorGraph::shared_ptr discreteGraph;

  if (continuous_ordering.size() > 0) {
    HybridEliminationTree continuous_etree(graph, etree_.variableIndex(),
                                           continuous_ordering);

    This continuous_junction_tree(continuous_etree);
    std::tie(continuousBayesTree, discreteGraph) =
        continuous_junction_tree.Base::eliminate(function);

  } else {
    continuousBayesTree = boost::make_shared<HybridBayesTree>();
    discreteGraph = boost::make_shared<HybridGaussianFactorGraph>(graph);
  }

  return std::make_pair(continuousBayesTree, discreteGraph);
}
/* ************************************************************************* */
std::pair<boost::shared_ptr<HybridBayesTree>,
          boost::shared_ptr<HybridGaussianFactorGraph>>
HybridJunctionTree::eliminateDiscrete(
    const Eliminate& function,
    const HybridBayesTree::shared_ptr& continuousBayesTree,
    const HybridGaussianFactorGraph::shared_ptr& discreteGraph,
    const Ordering& discrete_ordering) const {
  HybridBayesTree::shared_ptr updatedBayesTree;
  HybridGaussianFactorGraph::shared_ptr finalGraph;
  if (discrete_ordering.size() > 0) {
    HybridEliminationTree discrete_etree(
        *discreteGraph, VariableIndex(*discreteGraph), discrete_ordering);

    This discrete_junction_tree(discrete_etree);

    std::tie(updatedBayesTree, finalGraph) =
        discrete_junction_tree.Base::eliminate(function);

    // Get the clique with all the discrete keys.
    // There should only be 1 clique.
    const HybridBayesTree::sharedClique discrete_clique =
        (*updatedBayesTree)[discrete_ordering.at(0)];

    std::set<HybridBayesTreeClique::shared_ptr> clique_set;
    for (auto node : continuousBayesTree->nodes()) {
      clique_set.insert(node.second);
    }

    // Set the root of the bayes tree as the discrete clique
    for (auto clique : clique_set) {
      if (clique->conditional()->parents() ==
          discrete_clique->conditional()->frontals()) {
        updatedBayesTree->addClique(clique, discrete_clique);

      } else {
        if (clique->parent()) {
          // Remove the clique from the children of the parents since it will
          // get added again in addClique.
          auto clique_it = std::find(clique->parent()->children.begin(),
                                     clique->parent()->children.end(), clique);
          clique->parent()->children.erase(clique_it);
          updatedBayesTree->addClique(clique, clique->parent());
        } else {
          updatedBayesTree->addClique(clique);
        }
      }
    }
  } else {
    updatedBayesTree = continuousBayesTree;
    finalGraph = discreteGraph;
  }

  return std::make_pair(updatedBayesTree, finalGraph);
}

/* ************************************************************************* */
boost::shared_ptr<HybridGaussianFactorGraph> HybridJunctionTree::addProbPrimes(
    const HybridGaussianFactorGraph& graph,
    const HybridBayesTree::shared_ptr& continuousBayesTree,
    const HybridGaussianFactorGraph::shared_ptr& discreteGraph,
    const Ordering& continuous_ordering,
    const Ordering& discrete_ordering) const {
  // If we have eliminated continuous variables
  // and have discrete variables to eliminate,
  // then compute P(X | M, Z)
  if (continuous_ordering.size() > 0 && discrete_ordering.size() > 0) {
    // Collect all the discrete keys
    DiscreteKeys discrete_keys;
    for (auto node : continuousBayesTree->nodes()) {
      auto node_dkeys = node.second->conditional()->discreteKeys();
      discrete_keys.insert(discrete_keys.end(), node_dkeys.begin(),
                           node_dkeys.end());
    }
    // Remove duplicates and convert back to DiscreteKeys
    std::set<DiscreteKey> dkeys_set(discrete_keys.begin(), discrete_keys.end());
    discrete_keys = DiscreteKeys(dkeys_set.begin(), dkeys_set.end());

    // DecisionTree for P'(X|M, Z) for all mode sequences M
    const AlgebraicDecisionTree<Key> probPrimeTree =
        graph.continuousProbPrimes(discrete_keys, continuousBayesTree);

    // Add the model selection factor P(M|Z)
    discreteGraph->add(DecisionTreeFactor(discrete_keys, probPrimeTree));
  }

  return discreteGraph;
}

/* ************************************************************************* */
std::pair<HybridBayesTree::shared_ptr, HybridGaussianFactorGraph::shared_ptr>
HybridJunctionTree::eliminate(const Eliminate& function) const {
  Ordering continuous_ordering = etree_.continuousOrdering();
  Ordering discrete_ordering = etree_.discreteOrdering();

  FactorGraphType graph = etree_.graph();

  // Eliminate continuous
  BayesTreeType::shared_ptr continuousBayesTree;
  FactorGraphType::shared_ptr discreteGraph;
  std::tie(continuousBayesTree, discreteGraph) =
      this->eliminateContinuous(function, graph, continuous_ordering);

  FactorGraphType::shared_ptr updatedDiscreteGraph =
      this->addProbPrimes(graph, continuousBayesTree, discreteGraph,
                          continuous_ordering, discrete_ordering);

  // Eliminate discrete variables to get the discrete bayes tree.
  return this->eliminateDiscrete(function, continuousBayesTree,
                                 updatedDiscreteGraph, discrete_ordering);
}

}  // namespace gtsam
