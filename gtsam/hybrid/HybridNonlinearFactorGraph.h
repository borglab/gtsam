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

#include <gtsam/hybrid/HybridFactorGraph.h>

namespace gtsam {

class HybridGaussianFactorGraph;

/**
 * Nonlinear Hybrid Factor Graph
 * -----------------------
 * This is the non-linear version of a hybrid factor graph.
 * Everything inside needs to be hybrid factor or hybrid conditional.
 */
class GTSAM_EXPORT HybridNonlinearFactorGraph : public HybridFactorGraph {
 protected:
 public:
  using Base = HybridFactorGraph;
  using This = HybridNonlinearFactorGraph;     ///< this class
  using shared_ptr = std::shared_ptr<This>;  ///< shared_ptr to This

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
  /// @name Constructors
  /// @{

  /// Print the factor graph.
  void print(
      const std::string& s = "HybridNonlinearFactorGraph",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * @brief Linearize all the continuous factors in the
   * HybridNonlinearFactorGraph.
   *
   * @param continuousValues: Dictionary of continuous values.
   * @return HybridGaussianFactorGraph::shared_ptr
   */
  std::shared_ptr<HybridGaussianFactorGraph> linearize(
      const Values& continuousValues) const;
  /// @}
};

template <>
struct traits<HybridNonlinearFactorGraph>
    : public Testable<HybridNonlinearFactorGraph> {};

}  // namespace gtsam
