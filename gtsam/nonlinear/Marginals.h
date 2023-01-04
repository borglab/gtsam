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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

class JointMarginal;

/**
 * A class for computing Gaussian marginals of variables in a NonlinearFactorGraph
 */
class GTSAM_EXPORT Marginals {

public:

  /** The linear factorization mode - either CHOLESKY (faster and suitable for most problems) or QR (slower but more numerically stable for poorly-conditioned problems). */
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
   * @param factorization The linear decomposition mode - either Marginals::CHOLESKY (faster and suitable for most problems) or Marginals::QR (slower but more numerically stable for poorly-conditioned problems).
   */
  Marginals(const NonlinearFactorGraph& graph, const Values& solution, Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a nonlinear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The linearization point about which to compute Gaussian marginals (usually the MLE as obtained from a NonlinearOptimizer).
   * @param factorization The linear decomposition mode - either Marginals::CHOLESKY (faster and suitable for most problems) or Marginals::QR (slower but more numerically stable for poorly-conditioned problems).
   * @param ordering The ordering for elimination.
   */
  Marginals(const NonlinearFactorGraph& graph, const Values& solution, const Ordering& ordering,
              Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode - either Marginals::CHOLESKY (faster and suitable for most problems) or Marginals::QR (slower but more numerically stable for poorly-conditioned problems).
   */          
  Marginals(const GaussianFactorGraph& graph, const Values& solution, Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode - either Marginals::CHOLESKY (faster and suitable for most problems) or Marginals::QR (slower but more numerically stable for poorly-conditioned problems).
   * @param ordering The ordering for elimination.
   */          
  Marginals(const GaussianFactorGraph& graph, const Values& solution, const Ordering& ordering,
              Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode - either Marginals::CHOLESKY (faster and suitable for most problems) or Marginals::QR (slower but more numerically stable for poorly-conditioned problems).
   * @param ordering An optional variable ordering for elimination.
   */          
  Marginals(const GaussianFactorGraph& graph, const VectorValues& solution, Factorization factorization = CHOLESKY);

  /** Construct a marginals class from a linear factor graph.
   * @param graph The factor graph defining the full joint density on all variables.
   * @param solution The solution point to compute Gaussian marginals.
   * @param factorization The linear decomposition mode - either Marginals::CHOLESKY (faster and suitable for most problems) or Marginals::QR (slower but more numerically stable for poorly-conditioned problems).
   * @param ordering An optional variable ordering for elimination.
   */          
  Marginals(const GaussianFactorGraph& graph, const VectorValues& solution, const Ordering& ordering,
              Factorization factorization = CHOLESKY);

  /** print */
  void print(const std::string& str = "Marginals: ", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Compute the marginal factor of a single variable */
  GaussianFactor::shared_ptr marginalFactor(Key variable) const;

  /** Compute the marginal information matrix of a single variable.
    * Use LLt(const Matrix&) or RtR(const Matrix&) to obtain the square-root information matrix. */
  Matrix marginalInformation(Key variable) const;

  /** Compute the marginal covariance of a single variable */
  Matrix marginalCovariance(Key variable) const;

  /** Compute the joint marginal covariance of several variables */
  JointMarginal jointMarginalCovariance(const KeyVector& variables) const;

  /** Compute the joint marginal information of several variables */
  JointMarginal jointMarginalInformation(const KeyVector& variables) const;

  /** Optimize the bayes tree */
  VectorValues optimize() const;

protected:
  
  /** Compute the Bayes Tree as a helper function to the constructor */
  void computeBayesTree();

  /** Compute the Bayes Tree as a helper function to the constructor */
  void computeBayesTree(const Ordering& ordering);

public:
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  /** @deprecated argument order changed due to removing boost::optional<Ordering> */
  GTSAM_DEPRECATED Marginals(const NonlinearFactorGraph& graph, const Values& solution, Factorization factorization,
            const Ordering& ordering) : Marginals(graph, solution, ordering, factorization) {}

  /** @deprecated argument order changed due to removing boost::optional<Ordering> */
  GTSAM_DEPRECATED Marginals(const GaussianFactorGraph& graph, const Values& solution, Factorization factorization,
            const Ordering& ordering) : Marginals(graph, solution, ordering, factorization) {}

  /** @deprecated argument order changed due to removing boost::optional<Ordering> */
  GTSAM_DEPRECATED Marginals(const GaussianFactorGraph& graph, const VectorValues& solution, Factorization factorization,
            const Ordering& ordering) : Marginals(graph, solution, ordering, factorization) {}
#endif

};

/**
 * A class to store and access a joint marginal, returned from Marginals::jointMarginalCovariance and Marginals::jointMarginalInformation
 */
class GTSAM_EXPORT JointMarginal {

protected:
  SymmetricBlockMatrix blockMatrix_;
  KeyVector keys_;
  FastMap<Key, size_t> indices_;

public:
  /// Default constructor only for wrappers
  JointMarginal() {}

  /** Access a block, corresponding to a pair of variables, of the joint
   * marginal.  Each block is accessed by its "vertical position",
   * corresponding to the variable with nonlinear Key \c iVariable and
   * "horizontal position", corresponding to the variable with nonlinear Key
   * \c jVariable.
   *
   * For example, if we have the joint marginal on a 2D pose "x3" and a 2D
   * landmark "l2", then jointMarginal(Symbol('x',3), Symbol('l',2)) will
   * return the 3x2 block of the joint covariance matrix corresponding to x3
   * and l2.
   * @param iVariable The nonlinear Key specifying the "vertical position" of the requested block
   * @param jVariable The nonlinear Key specifying the "horizontal position" of the requested block
   */
  Matrix operator()(Key iVariable, Key jVariable) const {
    const auto indexI = indices_.at(iVariable);
    const auto indexJ = indices_.at(jVariable);
    return blockMatrix_.block(indexI, indexJ);
  }

  /** Synonym for operator() */
  Matrix at(Key iVariable, Key jVariable) const {
    return (*this)(iVariable, jVariable);
  }

  /** The full, dense covariance/information matrix of the joint marginal. */
  Matrix fullMatrix() const {
    return blockMatrix_.selfadjointView();
  }

  /** Print */
  void print(const std::string& s = "", const KeyFormatter& formatter = DefaultKeyFormatter) const;

protected:
  JointMarginal(const Matrix& fullMatrix, const std::vector<size_t>& dims, const KeyVector& keys) :
    blockMatrix_(dims, fullMatrix), keys_(keys), indices_(Ordering(keys).invert()) {}

  friend class Marginals;

};

} /* namespace gtsam */
