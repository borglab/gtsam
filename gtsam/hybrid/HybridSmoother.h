/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridSmoother.h
 * @brief   An incremental smoother for hybrid factor graphs
 * @author  Varun Agrawal
 * @date    October 2022
 */

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>

#include <optional>

namespace gtsam {

class GTSAM_EXPORT HybridSmoother {
 private:
  HybridBayesNet hybridBayesNet_;
  HybridGaussianFactorGraph remainingFactorGraph_;

 public:
  /**
   * Given new factors, perform an incremental update.
   * The relevant densities in the `hybridBayesNet` will be added to the input
   * graph (fragment), and then eliminated according to the `ordering`
   * presented. The remaining factor graph contains Gaussian mixture factors
   * that are not connected to the variables in the ordering, or a single
   * discrete factor on all discrete keys, plus all discrete factors in the
   * original graph.
   *
   * \note If maxComponents is given, we look at the discrete factor resulting
   * from this elimination, and prune it and the Gaussian components
   * corresponding to the pruned choices.
   *
   * @param graph The new factors, should be linear only
   * @param maxNrLeaves The maximum number of leaves in the new discrete factor,
   * if applicable
   * @param given_ordering The (optional) ordering for elimination, only
   * continuous variables are allowed
   */
  void update(HybridGaussianFactorGraph graph,
              std::optional<size_t> maxNrLeaves = {},
              const std::optional<Ordering> given_ordering = {});

  Ordering getOrdering(const HybridGaussianFactorGraph& newFactors);

  /**
   * @brief Add conditionals from previous timestep as part of liquefication.
   *
   * @param graph The new factor graph for the current time step.
   * @param hybridBayesNet The hybrid bayes net containing all conditionals so
   * far.
   * @param ordering The elimination ordering.
   * @return std::pair<HybridGaussianFactorGraph, HybridBayesNet>
   */
  std::pair<HybridGaussianFactorGraph, HybridBayesNet> addConditionals(
      const HybridGaussianFactorGraph& graph,
      const HybridBayesNet& hybridBayesNet, const Ordering& ordering) const;

  /// Get the Gaussian Mixture from the Bayes Net posterior at `index`.
  GaussianMixture::shared_ptr gaussianMixture(size_t index) const;

  /// Return the Bayes Net posterior.
  const HybridBayesNet& hybridBayesNet() const;
};

}  // namespace gtsam
