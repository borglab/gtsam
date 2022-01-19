/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianHybridFactorGraph.h
 * @brief  Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/hybrid/DCFactorGraph.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

namespace gtsam {

// Forward declarations
class GaussianMixture;
class Dummy;
class HybridFactorGraph;
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

class GaussianHybridFactorGraph
    : protected FactorGraph<Factor>,
      public EliminateableFactorGraph<GaussianHybridFactorGraph> {
 public:
  using shared_ptr = boost::shared_ptr<GaussianHybridFactorGraph>;
  using Base = FactorGraph<Factor>;

 protected:
  // Separate internal factor graphs for different types of factors
  GaussianFactorGraph gaussianGraph_;
  DiscreteFactorGraph discreteGraph_;
  DCFactorGraph dcGraph_;

  /// Check if FACTOR type is derived from DiscreteFactor.
  template <typename FACTOR>
  using IsDiscrete = typename std::enable_if<
      std::is_base_of<DiscreteFactor, FACTOR>::value>::type;

  /// Check if FACTOR type is derived from DCFactor.
  template <typename FACTOR>
  using IsDC =
      typename std::enable_if<std::is_base_of<DCFactor, FACTOR>::value>::type;

  /// Check if FACTOR type is derived from GaussianFactor.
  template <typename FACTOR>
  using IsGaussian = typename std::enable_if<
      std::is_base_of<GaussianFactor, FACTOR>::value>::type;

 public:
  /// Default constructor
  GaussianHybridFactorGraph() = default;

  /**
   * @brief Construct a new Hybrid Factor Graph object.
   *
   * @param nonlinearGraph A factor graph with continuous factors.
   * @param discreteGraph A factor graph with only discrete factors.
   * @param dcGraph A DCFactorGraph containing DCFactors.
   */
  GaussianHybridFactorGraph(const GaussianFactorGraph& gaussianGraph,
                            const DiscreteFactorGraph& discreteGraph,
                            const DCFactorGraph& dcGraph)
      : gaussianGraph_(gaussianGraph),
        discreteGraph_(discreteGraph),
        dcGraph_(dcGraph) {
    Base::push_back(discreteGraph);
    Base::push_back(dcGraph);
    Base::push_back(gaussianGraph);
  }

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
    gaussianGraph_.push_back(gaussianFactor);
    Base::push_back(gaussianFactor);
  }

  /**
   * Add a discrete factor *pointer* to the internal discrete graph
   * @param discreteFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsDiscrete<FACTOR> push_discrete(
      const boost::shared_ptr<FACTOR>& discreteFactor) {
    discreteGraph_.push_back(discreteFactor);
    Base::push_back(discreteFactor);
  }

  /**
   * Add a discrete-continuous (DC) factor *pointer* to the internal DC graph
   * @param dcFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsDC<FACTOR> push_dc(const boost::shared_ptr<FACTOR>& dcFactor) {
    dcGraph_.push_back(dcFactor);
    Base::push_back(dcFactor);
  }

  /// delete emplace_shared.
  template <class FACTOR, class... Args>
  void emplace_shared(Args&&... args) = delete;

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsGaussian<FACTOR> emplace_gaussian(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_gaussian(factor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsDiscrete<FACTOR> emplace_discrete(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_discrete(factor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsDC<FACTOR> emplace_dc(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_dc(factor);
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
    }
    if (auto p = boost::dynamic_pointer_cast<DiscreteFactor>(sharedFactor)) {
      push_discrete(p);
    }
    if (auto p = boost::dynamic_pointer_cast<DCFactor>(sharedFactor)) {
      push_dc(p);
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
  const GaussianFactorGraph& gaussianGraph() const { return gaussianGraph_; }

  /**
   * Utility for retrieving the internal discrete factor graph
   * @return the member variable discreteGraph_
   */
  const DiscreteFactorGraph& discreteGraph() const { return discreteGraph_; }

  /**
   * Utility for retrieving the internal DC factor graph
   * @return the member variable dcGraph_
   */
  const DCFactorGraph& dcGraph() const { return dcGraph_; }

  /// The total number of factors in the Gaussian factor graph.
  size_t nrGaussianFactors() const { return gaussianGraph_.size(); }

  /**
   * @return true if all internal graphs of `this` are equal to those of
   * `other`
   */
  bool equals(const GaussianHybridFactorGraph& other, double tol = 1e-9) const;

  /// The total number of factors in the discrete factor graph.
  size_t nrDiscreteFactors() const { return discreteGraph_.size(); }

  /// The total number of factors in the DC factor graph.
  size_t nrDcFactors() const { return dcGraph_.size(); }

  /**
   * Clears all internal factor graphs
   * TODO(dellaert): Not loving this!
   */
  void clear();

  /// The total number of discrete keys in the factor graph.
  DiscreteKeys discreteKeys() const;

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
   * @brief Convert the DecisionTree of (Key, GaussianFactorGraph) to (Key,
   * Graph Error).
   *
   * @return DecisionTreeFactor::shared_ptr
   */
  DecisionTreeFactor::shared_ptr toDecisionTreeFactor() const;

  /// @}
};

template <>
struct traits<GaussianHybridFactorGraph>
    : public Testable<GaussianHybridFactorGraph> {};

}  // namespace gtsam
