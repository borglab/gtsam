/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianHybridFactorGraph.h
 * @brief  Custom hybrid factor graph for discrete + continuous factors
 * @author Varun Agrawal
 * @date   January 2022
 */

#pragma once

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <string>

namespace gtsam {

// Forward declarations
class GaussianMixture;
class Dummy;
class GaussianHybridFactorGraph;
class HybridEliminationTree;
class Ordering;

/** Main elimination function for HybridFactorGraph */
using sharedFactor = boost::shared_ptr<Factor>;
GTSAM_EXPORT std::pair<GaussianMixture::shared_ptr, sharedFactor>
EliminateHybrid(const GaussianHybridFactorGraph& factors, const Ordering& keys);

template <>
struct EliminationTraits<GaussianHybridFactorGraph> {
  typedef Factor FactorType;
  typedef GaussianHybridFactorGraph FactorGraphType;
  typedef GaussianMixture ConditionalType;
  typedef HybridBayesNet BayesNetType;
  typedef HybridEliminationTree EliminationTreeType;
  typedef HybridBayesNet BayesTreeType;
  typedef HybridEliminationTree JunctionTreeType;

  /// The function type that does a single elimination step on a variable.
  static std::pair<GaussianMixture::shared_ptr, sharedFactor> DefaultEliminate(
      const GaussianHybridFactorGraph& factors, const Ordering& ordering) {
    return EliminateHybrid(factors, ordering);
  }
};

class GTSAM_EXPORT GaussianHybridFactorGraph
    : public HybridFactorGraph<GaussianFactorGraph>,
      public EliminateableFactorGraph<GaussianHybridFactorGraph> {
 protected:
  /// Check if FACTOR type is derived from GaussianFactor.
  template <typename FACTOR>
  using IsGaussian = typename std::enable_if<
      std::is_base_of<GaussianFactor, FACTOR>::value>::type;

 public:
  using shared_ptr = boost::shared_ptr<GaussianHybridFactorGraph>;
  using Base = HybridFactorGraph<GaussianFactorGraph>;

  /// Default constructor
  GaussianHybridFactorGraph() = default;

  /**
   * @brief Construct a new Hybrid Factor Graph object.
   *
   * @param gaussianGraph A factor graph with continuous factors.
   * @param discreteGraph A factor graph with only discrete factors.
   * @param dcGraph A DCFactorGraph containing DCFactors.
   */
  GaussianHybridFactorGraph(const GaussianFactorGraph& gaussianGraph,
                            const DiscreteFactorGraph& discreteGraph,
                            const DCFactorGraph& dcGraph)
      : Base(gaussianGraph, discreteGraph, dcGraph) {}

  // Allow use of selected FactorGraph methods:
  using Base::empty;
  using Base::reserve;
  using Base::size;
  using Base::operator[];

  /**
   * Add a gaussian factor *pointer* to the internal gaussian factor graph
   * @param gaussianFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsGaussian<FACTOR> push_gaussian(
      const boost::shared_ptr<FACTOR>& gaussianFactor) {
    factorGraph_.push_back(gaussianFactor);
    Base::Base::push_back(gaussianFactor);
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
   * @tparam FACTOR The factor type template
   * @param sharedFactor The factor to add to this factor graph.
   */
  template <typename FACTOR>
  void push_back(const boost::shared_ptr<FACTOR>& sharedFactor) {
    if (auto p = boost::dynamic_pointer_cast<GaussianFactor>(sharedFactor)) {
      push_gaussian(p);
    } else {
      Base::push_back(sharedFactor);
    }
  }

  /** Constructor from iterator over factors (shared_ptr or plain objects) */
  template <typename ITERATOR>
  void push_back(ITERATOR firstFactor, ITERATOR lastFactor) {
    for (auto&& it = firstFactor; it != lastFactor; it++) {
      push_back(*it);
    }
  }

  /**
   * Simply prints the factor graph.
   */
  void print(
      const std::string& str = "GaussianHybridFactorGraph",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /**
   * Utility for retrieving the internal gaussian factor graph
   * @return the member variable gaussianGraph_
   */
  const GaussianFactorGraph& gaussianGraph() const { return factorGraph_; }

  /// The total number of factors in the Gaussian factor graph.
  size_t nrGaussianFactors() const { return factorGraph_.size(); }

  /// @name Elimination machinery
  /// @{
  using FactorType = Factor;
  using EliminationResult =
      std::pair<boost::shared_ptr<GaussianMixture>, sharedFactor>;
  using Eliminate = std::function<EliminationResult(
      const GaussianHybridFactorGraph&, const Ordering&)>;

  /**
   * @brief Sum all gaussians and Gaussian mixtures together.
   * @return a decision tree of GaussianFactorGraphs
   *
   * Takes all factors, which *must* be all DCGaussianMixtureFactors or
   * GaussianFactors, and "add" them. This might involve decision-trees of
   * different structure, and creating a different decision tree for Gaussians.
   */
  DCGaussianMixtureFactor::Sum sum() const;

  /**
   * @brief Convert the DecisionTree of (Key, GaussianFactorGraph) to
   * (Key, Graph probability).
   * @return DecisionTreeFactor::shared_ptr
   */
  DecisionTreeFactor::shared_ptr toDecisionTreeFactor() const;

  /// @}
};

template <>
struct traits<GaussianHybridFactorGraph>
    : public Testable<GaussianHybridFactorGraph> {};

}  // namespace gtsam
