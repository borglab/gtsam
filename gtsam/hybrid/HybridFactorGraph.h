/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.h
 * @brief  Nonlinear hybrid factor graph that uses type erasure
 * @author Varun Agrawal
 * @date   May 28, 2022
 */

#pragma once

#include <gtsam/hybrid/GaussianHybridFactorGraph.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Ordering.h>

namespace gtsam {

/**
 * Hybrid Factor Graph
 * -----------------------
 * This is the non-linear version of a hybrid factor graph.
 * Everything inside needs to be hybrid factor or hybrid conditional.
 */
class HybridFactorGraph : public FactorGraph<HybridFactor> {
 public:
  using Base = FactorGraph<HybridFactor>;
  using This = HybridFactorGraph;              ///< this class
  using shared_ptr = boost::shared_ptr<This>;  ///< shared_ptr to This

  using Values = gtsam::Values;  ///< backwards compatibility
  using Indices = KeyVector;     ///> map from keys to values

  /// @name Constructors
  /// @{

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
  using FactorGraph::add;
  using Base::operator[];

  /// Add a Jacobian factor to the factor graph.
  void add(JacobianFactor&& factor);

  /// Add a Jacobian factor as a shared ptr.
  void add(boost::shared_ptr<JacobianFactor> factor);

  /// Add a DecisionTreeFactor to the factor graph.
  void add(DecisionTreeFactor&& factor);

  /// Add a DecisionTreeFactor as a shared ptr.
  void add(boost::shared_ptr<DecisionTreeFactor> factor);

  /**
   * @brief Linearize all the continuous factors in the
   * NonlinearHybridFactorGraph.
   *
   * @param continuousValues: Dictionary of continuous values.
   * @return GaussianHybridFactorGraph::shared_ptr
   */
  GaussianHybridFactorGraph::shared_ptr linearize(
      const Values& continuousValues) const {
    // create an empty linear FG
    GaussianHybridFactorGraph::shared_ptr linearFG =
        boost::make_shared<GaussianHybridFactorGraph>();

    linearFG->reserve(size());

    // linearize all factors
    for (const sharedFactor& factor : factors_) {
      if (factor) {
        if (auto nf = boost::dynamic_pointer_cast<NonlinearFactor>) {
          (*linearFG) += factor->linearize(linearizationPoint);
        } else if (auto hf = boost::dynamic_pointer_cast<HybridFactor>) {
          if (hf->isContinuous()) {
          }
        }

      } else
        (*linearFG) += GaussianFactor::shared_ptr();
    }
  }
};

}  // namespace gtsam
