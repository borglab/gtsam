/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Marginals.h
 * @brief A class for computing marginals in a NonlinearFactorGraph
 * @author Richard Roberts
 * @date May 14, 2012
 */

#pragma once

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/JointMarginal.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

/**
 * A class for computing Gaussian marginals of variables in a NonlinearFactorGraph
 */
class GTSAM_EXPORT Marginals {

 public:
  /** The linear factorization mode - either CHOLESKY (faster and suitable for
   * most problems) or QR (slower but more numerically stable for
   * poorly-conditioned problems). */
  enum Factorization {
    CHOLESKY,
    QR
  };

 protected:
  GaussianFactorGraph graph_;
  Values values_;
  Factorization factorization_;
  GaussianBayesTree bayesTree_;

 public:

  /// Default constructor only for wrappers
  Marginals(){}

  /** Construct a marginals class from a nonlinear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The linearization point about which to compute Gaussian marginals (usually the MLE as obtained from a NonlinearOptimizer).
   * @param factorization The linear decomposition mode: CHOLESKY|QR
   */
  Marginals(const NonlinearFactorGraph& graph, const Values& solution,
            Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a nonlinear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The linearization point about which to compute Gaussian marginals (usually the MLE as obtained from a NonlinearOptimizer).
   * @param factorization The linear decomposition mode: CHOLESKY|QR
   * @param ordering The ordering for elimination.
   */
  Marginals(const NonlinearFactorGraph& graph, const Values& solution, const Ordering& ordering,
              Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode: CHOLESKY|QR
   */
  Marginals(const GaussianFactorGraph& graph, const Values& solution, Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode: CHOLESKY|QR
   * @param ordering The ordering for elimination.
   */
  Marginals(const GaussianFactorGraph& graph, const Values& solution, const Ordering& ordering,
              Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode: CHOLESKY|QR
   * @param ordering An optional variable ordering for elimination.
   */
  Marginals(const GaussianFactorGraph& graph, const VectorValues& solution, Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode: CHOLESKY|QR
   * @param ordering An optional variable ordering for elimination.
   */
  Marginals(const GaussianFactorGraph& graph, const VectorValues& solution, const Ordering& ordering,
              Factorization factorization = CHOLESKY);

  /**
   * Construct a marginals class from a precomputed Bayes tree.
   * @param bayesTree The precomputed Gaussian Bayes tree representing the
   * factorization of the linear system.
   * @param solution The solution point at which to compute Gaussian marginals.
   * @param factorization The linear decomposition mode: CHOLESKY|QR
   */
  Marginals(GaussianBayesTree&& bayesTree, const VectorValues& solution,
            Factorization factorization = CHOLESKY);

  /** print */
  void print(const std::string& str = "Marginals: ", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Compute the marginal factor of a single variable */
  GaussianFactor::shared_ptr marginalFactor(Key variable) const;

  /// Compute the marginal information matrix of a single variable.
  Matrix marginalInformation(Key variable) const;

  /// Compute the marginal covariance of a single variable.
  Matrix marginalCovariance(Key variable) const;

  /// Compute the joint marginal covariance of several variables.
  JointMarginal jointMarginalCovariance(const KeyVector& variables) const;

  /// Compute the joint marginal information of several variables.
  JointMarginal jointMarginalInformation(const KeyVector& variables) const;

  /** Delete cached Bayes tree shortcuts created while computing marginals */
  void deleteCachedShortcuts();

  /** Optimize the bayes tree */
  VectorValues optimize() const;

 protected:
  /// Select the elimination rule that matches the requested marginal factorization.
  GaussianFactorGraph::Eliminate eliminationFunction() const;
  
  /** Compute the Bayes Tree as a helper function to the constructor */
  void computeBayesTree();

  /** Compute the Bayes Tree as a helper function to the constructor */
  void computeBayesTree(const Ordering& ordering);
};

} /* namespace gtsam */
