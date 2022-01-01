/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.h
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
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

namespace gtsam {

// Forward declarations
class DCConditional;
class Dummy;
class HybridFactorGraph;
class HybridEliminationTree;
class Ordering;

/** Main elimination function for HybridFactorGraph */
GTSAM_EXPORT std::pair<DCConditional::shared_ptr, boost::shared_ptr<Factor>>
EliminateHybrid(const HybridFactorGraph& factors, const Ordering& keys);

template <>
struct EliminationTraits<HybridFactorGraph> {
  typedef Factor FactorType;
  typedef HybridFactorGraph FactorGraphType;
  typedef DCConditional ConditionalType;
  typedef HybridBayesNet BayesNetType;
  typedef HybridEliminationTree EliminationTreeType;
  typedef HybridBayesNet BayesTreeType;
  typedef HybridEliminationTree JunctionTreeType;

  /// The function type that does a single elimination step on a variable.
  static std::pair<DCConditional::shared_ptr, boost::shared_ptr<Factor>>
  DefaultEliminate(const HybridFactorGraph& factors, const Ordering& ordering) {
    return EliminateHybrid(factors, ordering);
  }
};

class HybridFactorGraph : public FactorGraph<Factor>,
                          public EliminateableFactorGraph<HybridFactorGraph> {
 public:
  using shared_ptr = boost::shared_ptr<HybridFactorGraph>;

 protected:
  // Separate internal factor graphs for different types of factors
  NonlinearFactorGraph nonlinearGraph_;
  DiscreteFactorGraph discreteGraph_;
  DCFactorGraph dcGraph_;
  GaussianFactorGraph gaussianGraph_;

 public:
  /// Default constructor
  HybridFactorGraph() = default;

  /**
   * @brief Construct a new Hybrid Factor Graph object.
   *
   * @param nonlinearGraph A factor graph with continuous factors.
   * @param discreteGraph A factor graph with only discrete factors.
   * @param dcGraph A DCFactorGraph containing DCFactors.
   */
  HybridFactorGraph(
      const NonlinearFactorGraph& nonlinearGraph,
      const DiscreteFactorGraph& discreteGraph, const DCFactorGraph& dcGraph,
      const GaussianFactorGraph& gaussianGraph = GaussianFactorGraph())
      : nonlinearGraph_(nonlinearGraph),
        discreteGraph_(discreteGraph),
        dcGraph_(dcGraph),
        gaussianGraph_(gaussianGraph) {}

  /// Check if FACTOR type is derived from NonlinearFactor.
  template <typename FACTOR>
  using IsNonlinear = typename std::enable_if<
      std::is_base_of<NonlinearFactor, FACTOR>::value>::type;

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

  /**
   * Add a nonlinear factor *pointer* to the internal nonlinear factor graph
   * @param nonlinearFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsNonlinear<FACTOR> push_nonlinear(
      const boost::shared_ptr<FACTOR>& nonlinearFactor) {
    nonlinearGraph_.push_back(nonlinearFactor);
    push_back(nonlinearFactor);
  }

  /**
   * Add a discrete factor *pointer* to the internal discrete graph
   * @param discreteFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsDiscrete<FACTOR> push_discrete(
      const boost::shared_ptr<FACTOR>& discreteFactor) {
    discreteGraph_.push_back(discreteFactor);
    push_back(discreteFactor);
  }

  /**
   * Add a discrete-continuous (DC) factor *pointer* to the internal DC graph
   * @param dcFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsDC<FACTOR> push_dc(const boost::shared_ptr<FACTOR>& dcFactor) {
    dcGraph_.push_back(dcFactor);
    push_back(dcFactor);
  }

  /**
   * Add a gaussian factor *pointer* to the internal gaussian factor graph
   * @param gaussianFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsGaussian<FACTOR> push_gaussian(
      const boost::shared_ptr<FACTOR>& gaussianFactor) {
    gaussianGraph_.push_back(gaussianFactor);
    push_back(gaussianFactor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsNonlinear<FACTOR> emplace_shared(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_nonlinear(factor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsDiscrete<FACTOR> emplace_shared(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_discrete(factor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsDC<FACTOR> emplace_shared(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_dc(factor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsGaussian<FACTOR> emplace_shared(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_gaussian(factor);
  }

  // DEPRECATED below:.

  /**
   * Add a nonlinear factor to the internal nonlinear factor graph
   * @param nonlinearFactor - the factor to add
   */
  template <typename FACTOR>
  IsNonlinear<FACTOR> push_nonlinear(const FACTOR& nonlinearFactor) {
    emplace_shared<FACTOR>(nonlinearFactor);
  }

  /**
   * Add a discrete factor to the internal discrete graph
   * @param discreteFactor - the factor to add
   */
  template <typename FACTOR>
  IsDiscrete<FACTOR> push_discrete(const FACTOR& discreteFactor) {
    emplace_shared<FACTOR>(discreteFactor);
  }

  /**
   * Add a discrete-continuous (DC) factor to the internal DC graph
   * @param dcFactor - the factor to add
   */
  template <typename FACTOR>
  IsDC<FACTOR> push_dc(const FACTOR& dcFactor) {
    emplace_shared<FACTOR>(dcFactor);
  }

  /**
   * Simply prints the factor graph.
   */
  void print(
      const std::string& str = "HybridFactorGraph",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /**
   * Mimics the GTSAM::FactorGraph API: retrieve the keys from each internal
   * factor graph. Internally uses FastSet::merge(const FastSet &other) to
   * combine sets from the different member factor graphs.
   *
   * @return the (aggregate) set of keys in all of the internal factor graphs.
   */
  FastSet<Key> keys() const;

  /**
   * Utility for retrieving the internal nonlinear factor graph
   * @return the member variable nonlinearGraph_
   */
  const NonlinearFactorGraph& nonlinearGraph() const;

  /**
   * Utility for retrieving the internal discrete factor graph
   * @return the member variable discreteGraph_
   */
  const DiscreteFactorGraph& discreteGraph() const;

  /**
   * Utility for retrieving the internal gaussian factor graph
   * @return the member variable gaussianGraph_
   */
  const GaussianFactorGraph& gaussianGraph() const;

  /**
   * @brief Linearize all the continuous factors in the HybridFactorGraph.
   *
   * @param continuousValues: Dictionary of continuous values.
   * @return HybridFactorGraph
   */
  HybridFactorGraph linearize(const Values& continuousValues) const;

  /**
   * Utility for retrieving the internal DC factor graph
   * @return the member variable dcGraph_
   */
  const DCFactorGraph& dcGraph() const;

  /**
   * @return true if all internal graphs are empty
   */
  bool empty() const;

  /**
   * @return true if all internal graphs of `this` are equal to those of
   * `other`
   */
  bool equals(const HybridFactorGraph& other, double tol = 1e-9) const;

  /**
   * @return the total number of factors across all internal graphs
   */
  size_t size() const;

  /**
   * @return the total number of factors in the nonlinear factor graph
   */
  size_t size_nonlinear() const;

  /**
   * @return the total number of factors in the discrete factor graph
   */
  size_t size_discrete() const;

  /**
   * @return the total number of factors in the DC factor graph
   */
  size_t size_dc() const;

  /**
   * Clears all internal factor graphs
   */
  void clear();

  /// @name Elimination machinery
  /// @{
  using FactorType = Factor;
  using EliminationResult =
      std::pair<boost::shared_ptr<DCConditional>, boost::shared_ptr<Factor>>;
  using Eliminate = std::function<EliminationResult(const HybridFactorGraph&,
                                                    const Ordering&)>;
  /// @}
};

}  // namespace gtsam
