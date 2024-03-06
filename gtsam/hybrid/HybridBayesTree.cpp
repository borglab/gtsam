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
 * @author  Fan Jiang, Varun Agrawal
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

GaussianBayesNetTree& HybridBayesTree::addCliqueToTree(
    const sharedClique& clique, GaussianBayesNetTree& result) const {
  // Perform bottom-up inclusion
  for (sharedClique child : clique->children) {
    result = addCliqueToTree(child, result);
  }

  auto f = clique->conditional();

  if (auto hc = std::dynamic_pointer_cast<HybridConditional>(f)) {
    if (auto gm = hc->asMixture()) {
      result = gm->add(result);
    } else if (auto g = hc->asGaussian()) {
      result = addGaussian(result, g);
    } else {
      // Has to be discrete, which we don't add.
    }
  }
  return result;
}

/* ************************************************************************ */
GaussianBayesNetValTree HybridBayesTree::assembleTree() const {
  GaussianBayesNetTree result;
  for (auto&& root : roots_) {
    result = addCliqueToTree(root, result);
  }

  GaussianBayesNetValTree resultTree(result, [](const GaussianBayesNet& gbn) {
    return std::make_pair(gbn, 0.0);
  });
  return resultTree;
}

/* ************************************************************************* */
AlgebraicDecisionTree<Key> HybridBayesTree::modelSelection() const {
  /*
      To perform model selection, we need:
      q(mu; M, Z) * sqrt((2*pi)^n*det(Sigma))

      If q(mu; M, Z) = exp(-error) & k = 1.0 / sqrt((2*pi)^n*det(Sigma))
      thus, q * sqrt((2*pi)^n*det(Sigma)) = q/k = exp(log(q/k))
      = exp(log(q) - log(k)) = exp(-error - log(k))
      = exp(-(error + log(k))),
      where error is computed at the corresponding MAP point, gbt.error(mu).

      So we compute (error + log(k)) and exponentiate later
    */

  GaussianBayesNetValTree bnTree = assembleTree();

  GaussianBayesNetValTree bn_error = bnTree.apply(
      [this](const Assignment<Key>& assignment,
             const std::pair<GaussianBayesNet, double>& gbnAndValue) {
        //  Compute the X* of each assignment
        VectorValues mu = gbnAndValue.first.optimize();

        // mu is empty if gbn had nullptrs
        if (mu.size() == 0) {
          return std::make_pair(gbnAndValue.first,
                                std::numeric_limits<double>::max());
        }

        // Compute the error for X* and the assignment
        double error =
            this->error(HybridValues(mu, DiscreteValues(assignment)));

        return std::make_pair(gbnAndValue.first, error);
      });

  auto trees = unzip(bn_error);
  AlgebraicDecisionTree<Key> errorTree = trees.second;

  // Compute model selection term (with help from ADT methods)
  AlgebraicDecisionTree<Key> modelSelectionTerm = errorTree * -1;

  // Exponentiate using our scheme
  double max_log = modelSelectionTerm.max();
  modelSelectionTerm = DecisionTree<Key, double>(
      modelSelectionTerm,
      [&max_log](const double& x) { return std::exp(x - max_log); });
  modelSelectionTerm = modelSelectionTerm.normalize(modelSelectionTerm.sum());

  return modelSelectionTerm;
}

/* ************************************************************************* */
HybridValues HybridBayesTree::optimize() const {
  DiscreteFactorGraph discrete_fg;
  DiscreteValues mpe;

  // Compute model selection term
  AlgebraicDecisionTree<Key> modelSelectionTerm = modelSelection();

  auto root = roots_.at(0);
  // Access the clique and get the underlying hybrid conditional
  HybridConditional::shared_ptr root_conditional = root->conditional();

  // Get the set of all discrete keys involved in model selection
  std::set<DiscreteKey> discreteKeySet;

  //  The root should be discrete only, we compute the MPE
  if (root_conditional->isDiscrete()) {
    discrete_fg.push_back(root_conditional->asDiscrete());

    // Only add model_selection if we have discrete keys
    if (discreteKeySet.size() > 0) {
      discrete_fg.push_back(DecisionTreeFactor(
          DiscreteKeys(discreteKeySet.begin(), discreteKeySet.end()),
          modelSelectionTerm));
    }
    mpe = discrete_fg.optimize();
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
  // Flag indicating if all the nodes are valid. Used in optimize().
  bool valid_;

  /**
   * @brief Construct a new Hybrid Assignment Data object.
   *
   * @param assignment The MPE assignment for the optimal Gaussian cliques.
   * @param parentClique The clique from the parent node of the current node.
   * @param gbt The Gaussian Bayes Tree being generated during tree traversal.
   */
  HybridAssignmentData(const DiscreteValues& assignment,
                       const GaussianBayesTree::sharedNode& parentClique,
                       GaussianBayesTree* gbt, bool valid = true)
      : assignment_(assignment),
        parentClique_(parentClique),
        gaussianbayesTree_(gbt),
        valid_(valid) {}

  bool isValid() const { return valid_; }

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
      conditional = std::make_shared<GaussianConditional>();
    }

    GaussianBayesTree::sharedNode clique;
    if (conditional) {
      // Create the GaussianClique for the current node
      clique = std::make_shared<GaussianBayesTree::Node>(conditional);
      // Add the current clique to the GaussianBayesTree.
      parentData.gaussianbayesTree_->addClique(clique,
                                               parentData.parentClique_);
    } else {
      parentData.valid_ = false;
    }

    // Create new HybridAssignmentData where the current node is the parent
    // This will be passed down to the children nodes
    HybridAssignmentData data(parentData.assignment_, clique,
                              parentData.gaussianbayesTree_, parentData.valid_);
    return data;
  }
};

/* *************************************************************************
 */
GaussianBayesTree HybridBayesTree::choose(
    const DiscreteValues& assignment) const {
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

  if (!rootData.isValid()) {
    return GaussianBayesTree();
  }
  return gbt;
}

/* *************************************************************************
 */
VectorValues HybridBayesTree::optimize(const DiscreteValues& assignment) const {
  GaussianBayesTree gbt = this->choose(assignment);
  // If empty GaussianBayesTree, means a clique is pruned hence invalid
  if (gbt.size() == 0) {
    return VectorValues();
  }
  VectorValues result = gbt.optimize();

  // Return the optimized bayes net result.
  return result;
}

/* ************************************************************************* */
void HybridBayesTree::prune(const size_t maxNrLeaves) {
  auto discreteProbs = this->roots_.at(0)->conditional()->asDiscrete();

  DecisionTreeFactor prunedDiscreteProbs = discreteProbs->prune(maxNrLeaves);
  discreteProbs->root_ = prunedDiscreteProbs.root_;

  /// Helper struct for pruning the hybrid bayes tree.
  struct HybridPrunerData {
    /// The discrete decision tree after pruning.
    DecisionTreeFactor prunedDiscreteProbs;
    HybridPrunerData(const DecisionTreeFactor& prunedDiscreteProbs,
                     const HybridBayesTree::sharedNode& parentClique)
        : prunedDiscreteProbs(prunedDiscreteProbs) {}

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

        gaussianMixture->prune(parentData.prunedDiscreteProbs);
      }
      return parentData;
    }
  };

  HybridPrunerData rootData(prunedDiscreteProbs, 0);
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
