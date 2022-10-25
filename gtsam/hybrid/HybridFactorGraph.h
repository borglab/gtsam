/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.h
 * @brief  Hybrid factor graph base class that uses type erasure
 * @author Varun Agrawal
 * @date   May 28, 2022
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Ordering.h>

#include <boost/format.hpp>
namespace gtsam {

using SharedFactor = boost::shared_ptr<Factor>;

/**
 * Hybrid Factor Graph
 * -----------------------
 * This is the base hybrid factor graph.
 * Everything inside needs to be hybrid factor or hybrid conditional.
 */
class HybridFactorGraph : public FactorGraph<HybridFactor> {
 public:
  using Base = FactorGraph<HybridFactor>;
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
   * Add a discrete factor *pointer* to the internal discrete graph
   * @param discreteFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsDiscrete<FACTOR> push_discrete(
      const boost::shared_ptr<FACTOR>& discreteFactor) {
    Base::push_back(boost::make_shared<HybridDiscreteFactor>(discreteFactor));
  }

  /**
   * Add a discrete-continuous (Hybrid) factor *pointer* to the graph
   * @param hybridFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsHybrid<FACTOR> push_hybrid(const boost::shared_ptr<FACTOR>& hybridFactor) {
    Base::push_back(hybridFactor);
  }

  /// delete emplace_shared.
  template <class FACTOR, class... Args>
  void emplace_shared(Args&&... args) = delete;

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsDiscrete<FACTOR> emplace_discrete(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_discrete(factor);
  }

  /// Construct a factor and add (shared pointer to it) to factor graph.
  template <class FACTOR, class... Args>
  IsHybrid<FACTOR> emplace_hybrid(Args&&... args) {
    auto factor = boost::allocate_shared<FACTOR>(
        Eigen::aligned_allocator<FACTOR>(), std::forward<Args>(args)...);
    push_hybrid(factor);
  }

  /**
   * @brief Add a single factor shared pointer to the hybrid factor graph.
   * Dynamically handles the factor type and assigns it to the correct
   * underlying container.
   *
   * @param sharedFactor The factor to add to this factor graph.
   */
  void push_back(const SharedFactor& sharedFactor) {
    if (auto p = boost::dynamic_pointer_cast<DiscreteFactor>(sharedFactor)) {
      push_discrete(p);
    }
    if (auto p = boost::dynamic_pointer_cast<HybridFactor>(sharedFactor)) {
      push_hybrid(p);
    }
  }

  /// Get all the discrete keys in the factor graph.
  const KeySet discreteKeys() const {
    KeySet discrete_keys;
    for (auto& factor : factors_) {
      for (const DiscreteKey& k : factor->discreteKeys()) {
        discrete_keys.insert(k.first);
      }
    }
    return discrete_keys;
  }

  /// Get all the continuous keys in the factor graph.
  const KeySet continuousKeys() const {
    KeySet keys;
    for (auto& factor : factors_) {
      for (const Key& key : factor->continuousKeys()) {
        keys.insert(key);
      }
    }
    return keys;
  }
};

}  // namespace gtsam
