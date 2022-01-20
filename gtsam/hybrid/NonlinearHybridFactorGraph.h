/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   NonlinearHybridFactorGraph.h
 * @brief  Custom hybrid factor graph for discrete + nonlinear continuous
 * factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#pragma once

#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

namespace gtsam {

class GTSAM_EXPORT NonlinearHybridFactorGraph
    : public HybridFactorGraph<NonlinearFactorGraph> {
 public:
  using shared_ptr = boost::shared_ptr<NonlinearHybridFactorGraph>;
  using Base = HybridFactorGraph<NonlinearFactorGraph>;

 public:
  /// Default constructor
  NonlinearHybridFactorGraph() = default;

  /**
   * @brief Construct a new Hybrid Factor Graph object.
   *
   * @param nonlinearGraph A factor graph with continuous factors.
   * @param discreteGraph A factor graph with only discrete factors.
   * @param dcGraph A DCFactorGraph containing DCFactors.
   */
  NonlinearHybridFactorGraph(const NonlinearFactorGraph& nonlinearGraph,
                             const DiscreteFactorGraph& discreteGraph,
                             const DCFactorGraph& dcGraph)
      : Base(nonlinearGraph, discreteGraph, dcGraph) {}

  // Allow use of selected FactorGraph methods:
  using Base::empty;
  using Base::reserve;
  using Base::size;
  using Base::operator[];

  /**
   * Add a nonlinear factor *pointer* to the internal nonlinear factor graph
   * @param nonlinearFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsNonlinear<FACTOR> push_nonlinear(
      const boost::shared_ptr<FACTOR>& nonlinearFactor) {
    factorGraph_.push_back(nonlinearFactor);
    Base::Base::push_back(nonlinearFactor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsGaussian<FACTOR> emplace_gaussian(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_gaussian(factor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsNonlinear<FACTOR> emplace_nonlinear(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_nonlinear(factor);
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
    if (auto p = boost::dynamic_pointer_cast<NonlinearFactor>(sharedFactor)) {
      push_nonlinear(p);
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
      const std::string& str = "NonlinearHybridFactorGraph",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /**
   * Utility for retrieving the internal nonlinear factor graph
   * @return the member variable nonlinearGraph_
   */
  const gtsam::NonlinearFactorGraph& nonlinearGraph() const {
    return factorGraph_;
  }

  /**
   * @brief Linearize all the continuous factors in the
   * NonlinearHybridFactorGraph.
   *
   * @param continuousValues: Dictionary of continuous values.
   * @return GaussianHybridFactorGraph
   */
  GaussianHybridFactorGraph linearize(const Values& continuousValues) const;

  /**
   * @return true if all internal graphs of `this` are equal to those of
   * `other`
   */
  bool equals(const NonlinearHybridFactorGraph& other, double tol = 1e-9) const;

  /// The total number of factors in the nonlinear factor graph.
  size_t nrNonlinearFactors() const { return factorGraph_.size(); }

  /// @}
};

template <>
struct traits<NonlinearHybridFactorGraph>
    : public Testable<NonlinearHybridFactorGraph> {};

}  // namespace gtsam
