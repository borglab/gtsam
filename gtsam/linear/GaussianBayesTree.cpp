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
#include <gtsam/linear/VectorValues.h>

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
    double logDet;
    if (cg->get_model()) {
      Vector diag = cg->R().diagonal();
      cg->get_model()->whitenInPlace(diag);
      logDet = diag.unaryExpr([](double x) { return log(x); }).sum();
    } else {
      logDet =
          cg->R().diagonal().unaryExpr([](double x) { return log(x); }).sum();
    }
    // Add the current clique's log-determinant to the overall sum
    (*parentSum.logDet) += logDet;
    return parentSum;
  }
  }  // namespace internal

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
  Matrix GaussianBayesTree::marginalCovariance(Key key) const
  {
    return marginalFactor(key)->information().inverse();
  }


} // \namespace gtsam
