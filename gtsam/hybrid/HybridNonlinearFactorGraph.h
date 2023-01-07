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

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/format.hpp>
namespace gtsam {

/**
 * Nonlinear Hybrid Factor Graph
 * -----------------------
 * This is the non-linear version of a hybrid factor graph.
 * Everything inside needs to be hybrid factor or hybrid conditional.
 */
class GTSAM_EXPORT HybridNonlinearFactorGraph : public HybridFactorGraph {
 protected:
  /// Check if FACTOR type is derived from NonlinearFactor.
  template <typename FACTOR>
  using IsNonlinear = typename std::enable_if<
      std::is_base_of<NonlinearFactor, FACTOR>::value>::type;

  /// Check if T has a value_type derived from FactorType.
  template <typename T>
  using HasDerivedValueType = typename std::enable_if<
      std::is_base_of<HybridFactor, typename T::value_type>::value>::type;

 public:
  using Base = HybridFactorGraph;
  using This = HybridNonlinearFactorGraph;     ///< this class
  using shared_ptr = boost::shared_ptr<This>;  ///< shared_ptr to This

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

  // Allow use of selected FactorGraph methods:
  using Base::empty;
  using Base::reserve;
  using Base::size;
  using Base::operator[];
  using Base::add;
  using Base::resize;

  /**
   * Add a nonlinear factor *pointer* to the internal nonlinear factor graph
   * @param nonlinearFactor - boost::shared_ptr to the factor to add
   */
  template <typename FACTOR>
  IsNonlinear<FACTOR> push_nonlinear(
      const boost::shared_ptr<FACTOR>& nonlinearFactor) {
    Base::push_back(boost::make_shared<HybridNonlinearFactor>(nonlinearFactor));
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

  /**
   * Push back many factors as shared_ptr's in a container (factors are not
   * copied)
   */
  template <typename CONTAINER>
  void push_back(const CONTAINER& container) {
    Base::push_back(container.begin(), container.end());
  }

  /// Push back non-pointer objects in a container (factors are copied).
  template <typename CONTAINER>
  HasDerivedValueType<CONTAINER> push_back(const CONTAINER& container) {
    Base::push_back(container.begin(), container.end());
  }

  /// Add a nonlinear factor as a shared ptr.
  void add(boost::shared_ptr<NonlinearFactor> factor);

  /// Add a discrete factor as a shared ptr.
  void add(boost::shared_ptr<DiscreteFactor> factor);

  /// Print the factor graph.
  void print(
      const std::string& s = "HybridNonlinearFactorGraph",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /**
   * @brief Linearize all the continuous factors in the
   * HybridNonlinearFactorGraph.
   *
   * @param continuousValues: Dictionary of continuous values.
   * @return HybridGaussianFactorGraph::shared_ptr
   */
  HybridGaussianFactorGraph::shared_ptr linearize(
      const Values& continuousValues) const;
};

template <>
struct traits<HybridNonlinearFactorGraph>
    : public Testable<HybridNonlinearFactorGraph> {};

}  // namespace gtsam
