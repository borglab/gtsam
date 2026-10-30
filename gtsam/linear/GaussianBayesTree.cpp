/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTree.cpp
 * @brief   Gaussian Bayes Tree, the result of eliminating a GaussianJunctionTree
 * @brief   GaussianBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>
#include <gtsam/linear/linearAlgorithms-inst.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTreeQueries.h>
#include <gtsam/linear/VectorValues.h>

#include <map>
#include <stdexcept>

namespace gtsam {

  // Instantiate base class
  template class BayesTreeCliqueBase<GaussianBayesTreeClique, GaussianFactorGraph>;
  template class BayesTree<GaussianBayesTreeClique>;

  /* ************************************************************************ */
  namespace internal {

  /**
   * @brief Struct to help with traversing the Bayes Tree
   * for log-determinant computation.
   * Records the data which is passed to the child nodes in pre-order visit.
   */
  struct LogDeterminantData {
    // Use pointer so we can get the full result after tree traversal
    double* logDet;
    LogDeterminantData(double* logDet)
        : logDet(logDet) {}
  };
  /* ************************************************************************ */
  LogDeterminantData& logDeterminant(
      const GaussianBayesTreeClique::shared_ptr& clique,
      LogDeterminantData& parentSum) {
    auto cg = clique->conditional();
    double logDet = cg->logDeterminant();
    // Add the current clique's log-determinant to the overall sum
    (*parentSum.logDet) += logDet;
    return parentSum;
  }
  }  // namespace internal

  namespace {

  // Check only dimensions, pivots, and noise here. The triangular kernels check
  // their results, avoiding a scan or copy of all coefficient blocks per query.
  void validateSubstitutionConditional(const GaussianConditional& conditional) {
    if (conditional.nrFrontals() == 0 ||
        conditional.nrFrontals() > conditional.size() ||
        conditional.R().rows() != conditional.R().cols()) {
      throw std::invalid_argument(
          "GaussianBayesTree substitution requires square frontal blocks.");
    }
    const auto& model = conditional.get_model();
    if (model &&
        (model->isConstrained() || model->dim() != conditional.rows() ||
         !model->sigmasRef().allFinite() ||
         (model->sigmasRef().array() <= 0.0).any())) {
      throw std::invalid_argument(
          "GaussianBayesTree substitution requires finite positive diagonal noise.");
    }
    const auto frontalMatrix = conditional.R();
    const auto diagonal = frontalMatrix.diagonal();
    if (!diagonal.allFinite() || (diagonal.array() == 0.0).any()) {
      throw IndeterminateSystemException(conditional.keys().front());
    }
  }

  // Validate inputs for traversal and fill omitted frontal blocks with zeros.
  VectorValues substitutionRhs(const GaussianBayesTree& tree,
                               const VectorValues& rhs) {
    VectorValues complete;
    for (const auto& [key, clique] : tree.nodes()) {
      if (!clique || !clique->conditional())
        throw std::invalid_argument("GaussianBayesTree has a null conditional.");
      const auto& conditional = *clique->conditional();
      if (key != conditional.firstFrontalKey()) continue;
      validateSubstitutionConditional(conditional);
      for (auto frontal = conditional.beginFrontals();
           frontal != conditional.endFrontals(); ++frontal) {
        const DenseIndex dimension = conditional.getDim(frontal);
        if (dimension <= 0 ||
            !complete.emplace(*frontal, Vector::Zero(dimension)).second) {
          throw std::invalid_argument(
              "GaussianBayesTree has invalid or duplicate frontal variables.");
        }
      }
    }
    for (const auto& [key, value] : rhs) {
      auto destination = complete.find(key);
      if (destination == complete.end() ||
          destination->second.size() != value.size() || !value.allFinite()) {
        throw std::invalid_argument(
            "GaussianBayesTree rhs has an unknown key, wrong dimension, or nonfinite value.");
      }
      destination->second = value;
    }
    return complete;
  }

  using CliqueVectors = std::map<const GaussianBayesTreeClique*, VectorValues>;

  // Each clique owns a private frontal RHS and outgoing separator message. All
  // entries are allocated before traversal; parallel tasks never insert into it.
  CliqueVectors transposeWorkspace(const GaussianBayesTree& tree,
                                   const VectorValues& rhs) {
    CliqueVectors workspace;
    for (const auto& [key, clique] : tree.nodes()) {
      const auto& conditional = *clique->conditional();
      if (key != conditional.firstFrontalKey()) continue;
      auto& local = workspace[clique.get()];
      for (Key frontal : conditional.frontals())
        local.emplace(frontal, rhs.at(frontal));
      for (auto parent = conditional.beginParents();
           parent != conditional.endParents(); ++parent)
        local.emplace(*parent, Vector::Zero(conditional.getDim(parent)));
    }
    return workspace;
  }

  }  // namespace

  /* ************************************************************************* */
  bool GaussianBayesTree::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::optimize() const
  {
    return internal::linearAlgorithms::optimizeBayesTree(*this);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::backSubstitute(const VectorValues& rhs) const {
    const VectorValues complete = substitutionRhs(*this, rhs);
    return internal::linearAlgorithms::optimizeBayesTree(*this, &complete);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::backSubstituteTranspose(
      const VectorValues& rhs) const {
    VectorValues result = substitutionRhs(*this, rhs);
    CliqueVectors workspace = transposeWorkspace(*this, result);
    auto solveClique = [&workspace, &result](
                           const GaussianBayesTreeClique::shared_ptr& clique) {
      auto& local = workspace.at(clique.get());
      // A child's separator can include higher ancestors as well as this
      // clique's frontals. Retain both kinds of contribution in the local map.
      for (const auto& child : clique->children) {
        const auto& message = workspace.at(child.get());
        for (Key separator : child->conditional()->parents())
          local.at(separator) += message.at(separator);
      }
      const auto& conditional = *clique->conditional();
      conditional.solveTransposeInPlace(local);
      for (const auto& [key, value] : local) {
        if (!value.allFinite())
          throw IndeterminateSystemException(conditional.keys().front());
      }
      // Only this clique writes these frontals; ancestors read its separators.
      for (Key frontal : conditional.frontals())
        result.at(frontal).swap(local.at(frontal));
    };
    TbbOpenMPMixedScope threadLimiter;
    treeTraversal::PostOrderForestParallel(*this, solveClique);
    return result;
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::optimizeGradientSearch() const
  {
    gttic(GaussianBayesTree_optimizeGradientSearch);
    return GaussianFactorGraph(*this).optimizeGradientSearch();
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::gradient(const VectorValues& x0) const {
    return GaussianFactorGraph(*this).gradient(x0);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::gradientAtZero() const {
    return GaussianFactorGraph(*this).gradientAtZero();
  }

  /* ************************************************************************* */
  double GaussianBayesTree::error(const VectorValues& x) const {
    return GaussianFactorGraph(*this).error(x);
  }

  /* ************************************************************************* */
  double GaussianBayesTree::logDeterminant() const
  {
    if(this->roots_.empty()) {
      return 0.0;
    } else {
      double sum = 0.0;
      // Store the log-determinant in this struct.
      internal::LogDeterminantData rootData(&sum);
      // No need to do anything for post-operation.
      treeTraversal::no_op visitorPost;
      // Limits OpenMP threads if we're mixing TBB and OpenMP
      TbbOpenMPMixedScope threadLimiter;
      // Traverse the GaussianBayesTree depth first and call logDeterminant on each node.
      treeTraversal::DepthFirstForestParallel(*this, rootData, internal::logDeterminant, visitorPost);
      return sum;
    }
  }

  /* ************************************************************************* */
  double GaussianBayesTree::determinant() const
  {
    return exp(logDeterminant());
  }

  /* ************************************************************************* */
  Matrix GaussianBayesTree::marginalInformation(Key key) const {
    return marginalInformation(key, EliminatePreferCholesky);
  }

  /* ************************************************************************* */
  Matrix GaussianBayesTree::marginalCovariance(Key key) const {
    return marginalCovariance(key, EliminatePreferCholesky);
  }

  /* ************************************************************************* */
  JointMarginal GaussianBayesTree::jointMarginalInformation(
      const KeyVector& queryKeys) const {
    return jointMarginalInformation(queryKeys, EliminatePreferCholesky);
  }

  /* ************************************************************************* */
  JointMarginal GaussianBayesTree::jointMarginalCovariance(
      const KeyVector& queryKeys) const {
    return jointMarginalCovariance(queryKeys, EliminatePreferCholesky);
  }

} // \namespace gtsam
