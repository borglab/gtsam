/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactorGraph.h
 * @brief  Nonlinear hybrid factor graph that uses type erasure
 * @author Varun Agrawal
 * @date   May 28, 2022
 */

#pragma once

#include <gtsam/hybrid/HybridFactorGraph.h>

namespace gtsam {

class HybridGaussianFactorGraph;

/**
 * Nonlinear Hybrid Factor Graph
 * -----------------------
 * This is the non-linear version of a hybrid factor graph.
 * Everything inside needs to be hybrid factor or hybrid conditional.
 */
class GTSAM_EXPORT HybridNonlinearFactorGraph : public HybridFactorGraph {
 protected:
 public:
  using Base = HybridFactorGraph;
  using This = HybridNonlinearFactorGraph;   ///< this class
  using shared_ptr = std::shared_ptr<This>;  ///< shared_ptr to This

  using Values = gtsam::Values;  ///< backwards compatibility
  using Indices = KeyVector;     ///> map from keys to values

  /// @name Constructors
  /// @{

  HybridNonlinearFactorGraph() = default;

  /**
   * Implicit copy/downcast constructor to override explicit template container
   * constructor. In BayesTree this is used for:
   * `cachedSeparatorMarginal_.reset(*separatorMarginal)`
   * */
  template <class DERIVEDFACTOR>
  HybridNonlinearFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph)
      : Base(graph) {}

  /// @}
  /// @name Constructors
  /// @{

  /// Print the factor graph.
  void print(
      const std::string& s = "HybridNonlinearFactorGraph",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** print errors along with factors*/
  void printErrors(
      const HybridValues& values,
      const std::string& str = "HybridNonlinearFactorGraph: ",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter,
      const std::function<bool(const Factor* /*factor*/,
                               double /*whitenedError*/, size_t /*index*/)>&
          printCondition =
              [](const Factor*, double, size_t) { return true; }) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * @brief Linearize all the continuous factors in the
   * HybridNonlinearFactorGraph.
   *
   * @param continuousValues: Dictionary of continuous values.
   * @return HybridGaussianFactorGraph::shared_ptr
   */
  std::shared_ptr<HybridGaussianFactorGraph> linearize(
      const Values& continuousValues) const;

  /// Expose error(const HybridValues&) method.
  using Base::error;

  /**
   * @brief Compute error of (hybrid) nonlinear factors and discrete factors
   * over each discrete assignment, and return as a tree.
   *
   * Error \f$ e = \Vert f(x) - \mu \Vert_{\Sigma} \f$.
   *
   * @note: Gaussian and hybrid Gaussian factors are not considered!
   *
   * @param values Manifold values at which to compute the error.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> errorTree(const Values& values) const;

  /**
   * @brief Computer posterior P(M|X=x) when all continuous values X are given.
   * This is efficient as this simply takes -exp(.) of errorTree and normalizes.
   *
   * @note Not a DiscreteConditional as the cardinalities of the DiscreteKeys,
   * which we would need, are hard to recover.
   *
   * @param continuousValues Continuous values x to condition on.
   * @return DecisionTreeFactor
   */
  AlgebraicDecisionTree<Key> discretePosterior(
      const Values& continuousValues) const;

  /// @}
};

template <>
struct traits<HybridNonlinearFactorGraph>
    : public Testable<HybridNonlinearFactorGraph> {};

}  // namespace gtsam
