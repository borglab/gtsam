/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianFactorGraph.h
 * @brief  Linearized Hybrid factor graph that uses type erasure
 * @author Fan Jiang, Varun Agrawal, Frank Dellaert
 * @date   Mar 11, 2022
 */

#pragma once

#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/VectorValues.h>

#include <functional>
#include <optional>

namespace gtsam {

// Forward declarations
class HybridGaussianFactorGraph;
class HybridConditional;
class HybridBayesNet;
class HybridEliminationTree;
class HybridBayesTree;
class HybridJunctionTree;
class DecisionTreeFactor;
class TableFactor;
class JacobianFactor;
class HybridValues;

/**
 * @brief Main elimination function for HybridGaussianFactorGraph.
 *
 * @param factors The factor graph to eliminate.
 * @param keys The elimination ordering.
 * @return The conditional on the ordering keys and the remaining factors.
 * @ingroup hybrid
 */
GTSAM_EXPORT
std::pair<std::shared_ptr<HybridConditional>, std::shared_ptr<Factor>>
EliminateHybrid(const HybridGaussianFactorGraph& factors, const Ordering& keys);

/**
 * @brief Return a Colamd constrained ordering where the discrete keys are
 * eliminated after the continuous keys.
 *
 * @return const Ordering
 */
GTSAM_EXPORT const Ordering
HybridOrdering(const HybridGaussianFactorGraph& graph);

/* ************************************************************************* */
template <>
struct EliminationTraits<HybridGaussianFactorGraph> {
  typedef Factor FactorType;  ///< Type of factors in factor graph
  typedef HybridGaussianFactorGraph
      FactorGraphType;  ///< Type of the factor graph (e.g.
                        ///< HybridGaussianFactorGraph)
  typedef HybridConditional
      ConditionalType;  ///< Type of conditionals from elimination
  typedef HybridBayesNet
      BayesNetType;  ///< Type of Bayes net from sequential elimination
  typedef HybridEliminationTree
      EliminationTreeType;                      ///< Type of elimination tree
  typedef HybridBayesTree BayesTreeType;        ///< Type of Bayes tree
  typedef HybridJunctionTree JunctionTreeType;  ///< Type of Junction tree
  /// The default dense elimination function
  static std::pair<std::shared_ptr<ConditionalType>,
                   std::shared_ptr<FactorType>>
  DefaultEliminate(const FactorGraphType& factors, const Ordering& keys) {
    return EliminateHybrid(factors, keys);
  }
  /// The default ordering generation function
  static Ordering DefaultOrderingFunc(
      const FactorGraphType& graph,
      std::optional<std::reference_wrapper<const VariableIndex>>) {
    return HybridOrdering(graph);
  }
};

/**
 * Hybrid Gaussian Factor Graph
 * -----------------------
 * This is the linearized version of a hybrid factor graph.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridGaussianFactorGraph
    : public HybridFactorGraph,
      public EliminateableFactorGraph<HybridGaussianFactorGraph> {
 protected:
  /// Check if FACTOR type is derived from GaussianFactor.
  template <typename FACTOR>
  using IsGaussian = typename std::enable_if<
      std::is_base_of<GaussianFactor, FACTOR>::value>::type;

 public:
  using Base = HybridFactorGraph;
  using This = HybridGaussianFactorGraph;  ///< this class
  ///< for elimination
  using BaseEliminateable = EliminateableFactorGraph<This>;
  using shared_ptr = std::shared_ptr<This>;  ///< shared_ptr to This

  using Values = gtsam::Values;  ///< backwards compatibility
  using Indices = KeyVector;     ///< map from keys to values

  /// @name Constructors
  /// @{

  /// @brief Default constructor.
  HybridGaussianFactorGraph() = default;

  /** Construct from container of factors (shared_ptr or plain objects) */
  template <class CONTAINER>
  explicit HybridGaussianFactorGraph(const CONTAINER& factors)
      : Base(factors) {}

  /**
   * Implicit copy/downcast constructor to override explicit template container
   * constructor. In BayesTree this is used for:
   * `cachedSeparatorMarginal_.reset(*separatorMarginal)`
   * */
  template <class DERIVEDFACTOR>
  HybridGaussianFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph)
      : Base(graph) {}

  /// @}
  /// @name Testable
  /// @{

  // TODO(dellaert):  customize print and equals.
  // void print(
  //     const std::string& s = "HybridGaussianFactorGraph",
  //     const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  void printErrors(
      const HybridValues& values,
      const std::string& str = "HybridGaussianFactorGraph: ",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter,
      const std::function<bool(const Factor* /*factor*/,
                               double /*whitenedError*/, size_t /*index*/)>&
          printCondition =
              [](const Factor*, double, size_t) { return true; }) const;

  // bool equals(const This& fg, double tol = 1e-9) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /// Expose error(const HybridValues&) method.
  using Base::error;

  /**
   * @brief Compute error for each discrete assignment,
   * and return as a tree.
   *
   * Error \f$ e = \Vert x - \mu \Vert_{\Sigma} \f$.
   *
   * @param continuousValues Continuous values at which to compute the error.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> errorTree(
      const VectorValues& continuousValues) const;

  /**
   * @brief Compute unnormalized probability \f$ P(X | M, Z) \f$
   * for each discrete assignment, and return as a tree.
   *
   * @param continuousValues Continuous values at which to compute the
   * probability.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> probPrime(
      const VectorValues& continuousValues) const;

  /**
   * @brief Compute the unnormalized posterior probability for a continuous
   * vector values given a specific assignment.
   *
   * @return double
   */
  double probPrime(const HybridValues& values) const;

  /**
   * @brief Create a decision tree of factor graphs out of this hybrid factor
   * graph.
   *
   * For example, if there are two mixture factors, one with a discrete key A
   * and one with a discrete key B, then the decision tree will have two levels,
   * one for A and one for B. The leaves of the tree will be the Gaussian
   * factors that have only continuous keys.
   */
  GaussianFactorGraphTree assembleGraphTree() const;

  /// @}

  /// Get the GaussianFactorGraph at a given discrete assignment.
  GaussianFactorGraph operator()(const DiscreteValues& assignment) const;

};

}  // namespace gtsam
