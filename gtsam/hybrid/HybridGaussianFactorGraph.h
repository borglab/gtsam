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
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

// Forward declarations
class HybridGaussianFactorGraph;
class HybridConditional;
class HybridBayesNet;
class HybridEliminationTree;
class HybridBayesTree;
class HybridJunctionTree;
class DecisionTreeFactor;
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
std::pair<boost::shared_ptr<HybridConditional>, boost::shared_ptr<Factor>>
EliminateHybrid(const HybridGaussianFactorGraph& factors, const Ordering& keys);

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
  static std::pair<boost::shared_ptr<ConditionalType>,
                   boost::shared_ptr<FactorType>>
  DefaultEliminate(const FactorGraphType& factors, const Ordering& keys) {
    return EliminateHybrid(factors, keys);
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
  using BaseEliminateable =
      EliminateableFactorGraph<This>;          ///< for elimination
  using shared_ptr = boost::shared_ptr<This>;  ///< shared_ptr to This

  using Values = gtsam::Values;  ///< backwards compatibility
  using Indices = KeyVector;     ///< map from keys to values

  /// @name Constructors
  /// @{

  /// @brief Default constructor.
  HybridGaussianFactorGraph() = default;

  /**
   * Implicit copy/downcast constructor to override explicit template container
   * constructor. In BayesTree this is used for:
   * `cachedSeparatorMarginal_.reset(*separatorMarginal)`
   * */
  template <class DERIVEDFACTOR>
  HybridGaussianFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph)
      : Base(graph) {}

  /// @}
  /// @name Adding factors.
  /// @{

  using Base::add;
  using Base::push_back;
  using Base::reserve;

  /// Add a Jacobian factor to the factor graph.
  void add(JacobianFactor&& factor);

  /// Add a Jacobian factor as a shared ptr.
  void add(const boost::shared_ptr<JacobianFactor>& factor);

  /// Add a DecisionTreeFactor to the factor graph.
  void add(DecisionTreeFactor&& factor);

  /// Add a DecisionTreeFactor as a shared ptr.
  void add(const boost::shared_ptr<DecisionTreeFactor>& factor);

  /**
   * Add a gaussian factor *pointer* to the internal gaussian factor graph
   * @param gaussianFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsGaussian<FACTOR> push_gaussian(
      const boost::shared_ptr<FACTOR>& gaussianFactor) {
    Base::push_back(boost::make_shared<HybridGaussianFactor>(gaussianFactor));
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsGaussian<FACTOR> emplace_gaussian(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_gaussian(factor);
  }

  /**
   * @brief Add a single factor shared pointer to the hybrid factor graph.
   * Dynamically handles the factor type and assigns it to the correct
   * underlying container.
   *
   * @param sharedFactor The factor to add to this factor graph.
   */
  void push_back(const SharedFactor& sharedFactor) {
    if (auto p = boost::dynamic_pointer_cast<GaussianFactor>(sharedFactor)) {
      push_gaussian(p);
    } else {
      Base::push_back(sharedFactor);
    }
  }

  /// @}
  /// @name Testable
  /// @{

  // TODO(dellaert):  customize print and equals.
  // void print(const std::string& s = "HybridGaussianFactorGraph",
  //            const KeyFormatter& keyFormatter = DefaultKeyFormatter) const
  //     override;
  // bool equals(const This& fg, double tol = 1e-9) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  using Base::empty;
  using Base::size;
  using Base::operator[];
  using Base::resize;

  /**
   * @brief Compute error for each discrete assignment,
   * and return as a tree.
   *
   * Error \f$ e = \Vert x - \mu \Vert_{\Sigma} \f$.
   *
   * @param continuousValues Continuous values at which to compute the error.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> error(const VectorValues& continuousValues) const;

  /**
   * @brief Compute error given a continuous vector values
   * and a discrete assignment.
   *
   * @return double
   */
  double error(const HybridValues& values) const;

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
   * @brief Return a Colamd constrained ordering where the discrete keys are
   * eliminated after the continuous keys.
   *
   * @return const Ordering
   */
  const Ordering getHybridOrdering() const;

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
};

}  // namespace gtsam
