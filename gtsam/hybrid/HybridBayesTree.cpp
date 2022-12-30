/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesTree.cpp
 * @brief   Hybrid Bayes Tree, the result of eliminating a
 * HybridJunctionTree
 * @date Mar 11, 2022
 * @author  Fan Jiang
 */

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>
#include <gtsam/linear/GaussianJunctionTree.h>

namespace gtsam {

// Instantiate base class
template class BayesTreeCliqueBase<HybridBayesTreeClique,
                                   HybridGaussianFactorGraph>;
template class BayesTree<HybridBayesTreeClique>;

/* ************************************************************************* */
bool HybridBayesTree::equals(const This& other, double tol) const {
  return Base::equals(other, tol);
}

/* ************************************************************************* */
HybridValues HybridBayesTree::optimize() const {
  DiscreteBayesNet dbn;
  DiscreteValues mpe;

  auto root = roots_.at(0);
  // Access the clique and get the underlying hybrid conditional
  HybridConditional::shared_ptr root_conditional = root->conditional();

  // The root should be discrete only, we compute the MPE
  if (root_conditional->isDiscrete()) {
    dbn.push_back(root_conditional->asDiscrete());
    mpe = DiscreteFactorGraph(dbn).optimize();
  } else {
    throw std::runtime_error(
        "HybridBayesTree root is not discrete-only. Please check elimination "
        "ordering or use continuous factor graph.");
  }

  VectorValues values = optimize(mpe);
  return HybridValues(values, mpe);
}

/* ************************************************************************* */
/**
 * @brief Helper class for Depth First Forest traversal on the HybridBayesTree.
 *
 * When traversing the tree, the pre-order visitor will receive an instance of
 * this class with the parent clique data.
 */
struct HybridAssignmentData {
  const DiscreteValues assignment_;
  GaussianBayesTree::sharedNode parentClique_;
  // The gaussian bayes tree that will be recursively created.
  GaussianBayesTree* gaussianbayesTree_;

  /**
   * @brief Construct a new Hybrid Assignment Data object.
   *
   * @param assignment The MPE assignment for the optimal Gaussian cliques.
   * @param parentClique The clique from the parent node of the current node.
   * @param gbt The Gaussian Bayes Tree being generated during tree traversal.
   */
  HybridAssignmentData(const DiscreteValues& assignment,
                       const GaussianBayesTree::sharedNode& parentClique,
                       GaussianBayesTree* gbt)
      : assignment_(assignment),
        parentClique_(parentClique),
        gaussianbayesTree_(gbt) {}

  /**
   * @brief A function used during tree traversal that operates on each node
   * before visiting the node's children.
   *
   * @param node The current node being visited.
   * @param parentData The HybridAssignmentData from the parent node.
   * @return HybridAssignmentData which is passed to the children.
   */
  static HybridAssignmentData AssignmentPreOrderVisitor(
      const HybridBayesTree::sharedNode& node,
      HybridAssignmentData& parentData) {
    // Extract the gaussian conditional from the Hybrid clique
    HybridConditional::shared_ptr hybrid_conditional = node->conditional();
    GaussianConditional::shared_ptr conditional;
    if (hybrid_conditional->isHybrid()) {
      conditional = (*hybrid_conditional->asMixture())(parentData.assignment_);
    } else if (hybrid_conditional->isContinuous()) {
      conditional = hybrid_conditional->asGaussian();
    } else {
      // Discrete only conditional, so we set to empty gaussian conditional
      conditional = boost::make_shared<GaussianConditional>();
    }

    // Create the GaussianClique for the current node
    auto clique = boost::make_shared<GaussianBayesTree::Node>(conditional);
    // Add the current clique to the GaussianBayesTree.
    parentData.gaussianbayesTree_->addClique(clique, parentData.parentClique_);

    // Create new HybridAssignmentData where the current node is the parent
    // This will be passed down to the children nodes
    HybridAssignmentData data(parentData.assignment_, clique,
                              parentData.gaussianbayesTree_);
    return data;
  }
};

/* *************************************************************************
 */
VectorValues HybridBayesTree::optimize(const DiscreteValues& assignment) const {
  GaussianBayesTree gbt;
  HybridAssignmentData rootData(assignment, 0, &gbt);
  {
    treeTraversal::no_op visitorPost;
    // Limits OpenMP threads since we're mixing TBB and OpenMP
    TbbOpenMPMixedScope threadLimiter;
    treeTraversal::DepthFirstForestParallel(
        *this, rootData, HybridAssignmentData::AssignmentPreOrderVisitor,
        visitorPost);
  }

  VectorValues result = gbt.optimize();

  // Return the optimized bayes net result.
  return result;
}

/* ************************************************************************* */
void HybridBayesTree::prune(const size_t maxNrLeaves) {
  auto decisionTree =
      this->roots_.at(0)->conditional()->asDiscrete();

  DecisionTreeFactor prunedDecisionTree = decisionTree->prune(maxNrLeaves);
  decisionTree->root_ = prunedDecisionTree.root_;

  /// Helper struct for pruning the hybrid bayes tree.
  struct HybridPrunerData {
    /// The discrete decision tree after pruning.
    DecisionTreeFactor prunedDecisionTree;
    HybridPrunerData(const DecisionTreeFactor& prunedDecisionTree,
                     const HybridBayesTree::sharedNode& parentClique)
        : prunedDecisionTree(prunedDecisionTree) {}

    /**
     * @brief A function used during tree traversal that operates on each node
     * before visiting the node's children.
     *
     * @param node The current node being visited.
     * @param parentData The data from the parent node.
     * @return HybridPrunerData which is passed to the children.
     */
    static HybridPrunerData AssignmentPreOrderVisitor(
        const HybridBayesTree::sharedNode& clique,
        HybridPrunerData& parentData) {
      // Get the conditional
      HybridConditional::shared_ptr conditional = clique->conditional();

      // If conditional is hybrid, we prune it.
      if (conditional->isHybrid()) {
        auto gaussianMixture = conditional->asMixture();

        gaussianMixture->prune(parentData.prunedDecisionTree);
      }
      return parentData;
    }
  };

  HybridPrunerData rootData(prunedDecisionTree, 0);
  {
    treeTraversal::no_op visitorPost;
    // Limits OpenMP threads since we're mixing TBB and OpenMP
    TbbOpenMPMixedScope threadLimiter;
    treeTraversal::DepthFirstForestParallel(
        *this, rootData, HybridPrunerData::AssignmentPreOrderVisitor,
        visitorPost);
  }
}

}  // namespace gtsam
