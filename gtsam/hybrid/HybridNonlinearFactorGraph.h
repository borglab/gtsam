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

#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
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

  /** Constructor from iterator over factors (shared_ptr or plain objects) */
  template <typename ITERATOR>
  void push_back(ITERATOR firstFactor, ITERATOR lastFactor) {
    for (auto&& it = firstFactor; it != lastFactor; it++) {
      push_back(*it);
    }
  }

  // /// Add a nonlinear factor to the factor graph.
  // void add(NonlinearFactor&& factor) {
  //   Base::add(boost::make_shared<HybridNonlinearFactor>(std::move(factor)));
  // }

  /// Add a nonlinear factor as a shared ptr.
  void add(boost::shared_ptr<NonlinearFactor> factor) {
    FactorGraph::add(boost::make_shared<HybridNonlinearFactor>(factor));
  }

  /**
   * Simply prints the factor graph.
   */
  void print(
      const std::string& str = "HybridNonlinearFactorGraph",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {}

  /**
   * @return true if all internal graphs of `this` are equal to those of
   * `other`
   */
  bool equals(const HybridNonlinearFactorGraph& other,
              double tol = 1e-9) const {
    return false;
  }

  /**
   * @brief Linearize all the continuous factors in the
   * HybridNonlinearFactorGraph.
   *
   * @param continuousValues: Dictionary of continuous values.
   * @return HybridGaussianFactorGraph::shared_ptr
   */
  HybridGaussianFactorGraph linearize(const Values& continuousValues) const {
    // create an empty linear FG
    HybridGaussianFactorGraph linearFG;

    linearFG.reserve(size());

    // linearize all hybrid factors
    for (auto&& factor : factors_) {
      // First check if it is a valid factor
      if (factor) {
        // Check if the factor is a hybrid factor.
        // It can be either a nonlinear MixtureFactor or a linear
        // GaussianMixtureFactor.
        if (factor->isHybrid()) {
          // Check if it is a nonlinear mixture factor
          if (auto nlmf = boost::dynamic_pointer_cast<MixtureFactor>(factor)) {
            linearFG.push_back(nlmf->linearize(continuousValues));
          } else {
            linearFG.push_back(factor);
          }

          // Now check if the factor is a continuous only factor.
        } else if (factor->isContinuous()) {
          // In this case, we check if factor->inner() is nonlinear since
          // HybridFactors wrap over continuous factors.
          auto nlhf =
              boost::dynamic_pointer_cast<HybridNonlinearFactor>(factor);
          if (auto nlf =
                  boost::dynamic_pointer_cast<NonlinearFactor>(nlhf->inner())) {
            auto hgf = boost::make_shared<HybridGaussianFactor>(
                nlf->linearize(continuousValues));
            linearFG.push_back(hgf);
          } else {
            linearFG.push_back(factor);
          }
          // Finally if nothing else, we are discrete-only which doesn't need
          // lineariztion.
        } else {
          linearFG.push_back(factor);
        }

      } else {
        linearFG.push_back(GaussianFactor::shared_ptr());
      }
    }
    return linearFG;
  }
};

template <>
struct traits<HybridNonlinearFactorGraph>
    : public Testable<HybridNonlinearFactorGraph> {};

}  // namespace gtsam
