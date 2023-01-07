/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.h
 * @brief  Factor graph with utilities for hybrid factors.
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   May 28, 2022
 */

#pragma once

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/FactorGraph.h>

#include <boost/format.hpp>
#include <unordered_map>

namespace gtsam {

class DiscreteFactor;
class Ordering;

using SharedFactor = boost::shared_ptr<Factor>;

/**
 * Hybrid Factor Graph
 * Factor graph with utilities for hybrid factors.
 */
class HybridFactorGraph : public FactorGraph<Factor> {
 public:
  using Base = FactorGraph<Factor>;
  using This = HybridFactorGraph;              ///< this class
  using shared_ptr = boost::shared_ptr<This>;  ///< shared_ptr to This

  using Values = gtsam::Values;  ///< backwards compatibility
  using Indices = KeyVector;     ///> map from keys to values

 protected:
  /// Check if FACTOR type is derived from DiscreteFactor.
  template <typename FACTOR>
  using IsDiscrete = typename std::enable_if<
      std::is_base_of<DiscreteFactor, FACTOR>::value>::type;

  /// Check if FACTOR type is derived from HybridFactor.
  template <typename FACTOR>
  using IsHybrid = typename std::enable_if<
      std::is_base_of<HybridFactor, FACTOR>::value>::type;

 public:
  /// @name Constructors
  /// @{

  /// Default constructor
  HybridFactorGraph() = default;

  /**
   * Implicit copy/downcast constructor to override explicit template container
   * constructor. In BayesTree this is used for:
   * `cachedSeparatorMarginal_.reset(*separatorMarginal)`
   * */
  template <class DERIVEDFACTOR>
  HybridFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph) : Base(graph) {}

  /// @}

  // Allow use of selected FactorGraph methods:
  using Base::empty;
  using Base::reserve;
  using Base::size;
  using Base::operator[];
  using Base::add;
  using Base::push_back;
  using Base::resize;

  /**
   * Add a discrete-continuous (Hybrid) factor *pointer* to the graph
   * @param hybridFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsHybrid<FACTOR> push_hybrid(const boost::shared_ptr<FACTOR>& hybridFactor) {
    Base::push_back(hybridFactor);
  }

  /// Construct a discrete factor and add shared pointer to the factor graph.
  template <class FACTOR, class... Args>
  IsDiscrete<FACTOR> emplace_discrete(Args&&... args) {
    emplace_shared<FACTOR>(std::forward<Args>(args)...);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsHybrid<FACTOR> emplace_hybrid(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_hybrid(factor);
  }

  /// Get all the discrete keys in the factor graph.
  DiscreteKeys discreteKeys() const;

  /// Get all the discrete keys in the factor graph, as a set.
  KeySet discreteKeySet() const;

  /// Get a map from Key to corresponding DiscreteKey.
  std::unordered_map<Key, DiscreteKey> discreteKeyMap() const;

  /// Get all the continuous keys in the factor graph.
  const KeySet continuousKeySet() const;
};

}  // namespace gtsam
