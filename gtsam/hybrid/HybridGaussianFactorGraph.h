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

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
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

  /**
   * @brief Print the errors of each factor in the hybrid factor graph.
   *
   * @param values The HybridValues for the variables used to compute the error.
   * @param str String that is output before the factor graph and errors.
   * @param keyFormatter Formatter function for the keys in the factors.
   * @param printCondition A condition to check if a factor should be printed.
   */
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
   * @brief Compute the unnormalized posterior probability for a continuous
   * vector values given a specific assignment.
   *
   * @return double
   */
  double probPrime(const HybridValues& values) const;

  /**
   * @brief Computer posterior P(M|X=x) when all continuous values X are given.
   * This is efficient as this simply probPrime normalized.
   *
   * @note Not a DiscreteConditional as the cardinalities of the DiscreteKeys,
   * which we would need, are hard to recover.
   *
   * @param continuousValues Continuous values x to condition on.
   * @return DecisionTreeFactor
   */
  AlgebraicDecisionTree<Key> discretePosterior(
      const VectorValues& continuousValues) const;

  /**
   * @brief Create a decision tree of factor graphs out of this hybrid factor
   * graph.
   *
   * For example, if there are two hybrid factors, one with a discrete key A
   * and one with a discrete key B, then the decision tree will have two levels,
   * one for A and one for B. The leaves of the tree will be the Gaussian
   * factors that have only continuous keys.
   */
  HybridGaussianProductFactor collectProductFactor() const;

  /**
   * @brief Eliminate the given continuous keys.
   *
   * @param keys The continuous keys to eliminate.
   * @return The conditional on the  keys and a factor on the separator.
   */
  std::pair<std::shared_ptr<HybridConditional>, std::shared_ptr<Factor>>
  eliminate(const Ordering& keys) const;
  /// @}

  /**
   @brief Get the GaussianFactorGraph at a given discrete assignment. Note this
   * corresponds to the Gaussian posterior p(X|M=m, Z=z) of the continuous
   * variables X given the discrete assignment M=m and whatever measurements z
   * where assumed in the creation of the factor Graph.
   *
   * @note Be careful, as any factors not Gaussian are ignored.
   *
   * @param assignment The discrete value assignment for the discrete keys.
   * @return Gaussian factors as a GaussianFactorGraph
   */
  GaussianFactorGraph choose(const DiscreteValues& assignment) const;

  /// Syntactic sugar for choose
  GaussianFactorGraph operator()(const DiscreteValues& assignment) const {
    return choose(assignment);
  }
};

// traits
template <>
struct traits<HybridGaussianFactorGraph>
    : public Testable<HybridGaussianFactorGraph> {};

}  // namespace gtsam
