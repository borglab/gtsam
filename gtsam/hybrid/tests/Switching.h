/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file Switching.h
 *  @date Mar 11, 2022
 *  @author Varun Agrawal
 *  @author Fan Jiang
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#pragma once

namespace gtsam {

using symbol_shorthand::C;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ****************************************************************************/
// Test fixture with switching network.
// TODO(Varun) Currently this is only linear. We need to add nonlinear support
// and then update to
// https://github.com/borglab/gtsam/pull/973/files#diff-58c02b3b197ebf731694946e87762d252e9eaa2f5c6c4ba22d618085b321ca23
struct Switching {
  size_t K;
  DiscreteKeys modes;
  HybridGaussianFactorGraph linearizedFactorGraph;
  Values linearizationPoint;

  using MotionModel = BetweenFactor<double>;
  // using MotionMixture = MixtureFactor<MotionModel>;

  /// Create with given number of time steps.
  Switching(size_t K, double between_sigma = 1.0, double prior_sigma = 0.1)
      : K(K) {
    // Create DiscreteKeys for binary K modes, modes[0] will not be used.
    modes = addDiscreteModes(K);

    // Create hybrid factor graph.
    // Add a prior on X(1).
    auto prior = boost::make_shared<JacobianFactor>(
        X(1), Matrix11::Ones() / prior_sigma, Vector1::Zero());
    linearizedFactorGraph.push_back(prior);

    // Add "motion models".
    linearizedFactorGraph = addMotionModels(K);

    // Add measurement factors
    for (size_t k = 1; k <= K; k++) {
      linearizedFactorGraph.emplace_gaussian<JacobianFactor>(
          X(k), Matrix11::Ones() / 0.1, Vector1::Zero());
    }

    // Add "mode chain"
    linearizedFactorGraph = addModeChain(linearizedFactorGraph);

    // Create the linearization point.
    for (size_t k = 1; k <= K; k++) {
      linearizationPoint.insert<double>(X(k), static_cast<double>(k));
    }
  }

  /// Create DiscreteKeys for K binary modes.
  DiscreteKeys addDiscreteModes(size_t K) {
    DiscreteKeys m;
    for (size_t k = 0; k <= K; k++) {
      m.emplace_back(M(k), 2);
    }
    return m;
  }

  /// Helper function to add motion models for each [k, k+1] interval.
  HybridGaussianFactorGraph addMotionModels(size_t K) {
    HybridGaussianFactorGraph hgfg;
    for (size_t k = 1; k < K; k++) {
      auto keys = {X(k), X(k + 1)};
      auto components = motionModels(k);
      hgfg.emplace_hybrid<GaussianMixtureFactor>(keys, DiscreteKeys{modes[k]},
                                                 components);
    }
    return hgfg;
  }

  // Create motion models for a given time step
  static std::vector<GaussianFactor::shared_ptr> motionModels(
      size_t k, double sigma = 1.0) {
    auto noise_model = noiseModel::Isotropic::Sigma(1, sigma);
    auto still = boost::make_shared<JacobianFactor>(
             X(k), -Matrix11::Ones() / sigma, X(k + 1),
             Matrix11::Ones() / sigma, Vector1::Zero()),
         moving = boost::make_shared<JacobianFactor>(
             X(k), -Matrix11::Ones() / sigma, X(k + 1),
             Matrix11::Ones() / sigma, -Vector1::Ones() / sigma);
    return {boost::dynamic_pointer_cast<GaussianFactor>(still),
            boost::dynamic_pointer_cast<GaussianFactor>(moving)};
  }

  // // Add "mode chain" to NonlinearHybridFactorGraph
  // void addModeChain(HybridNonlinearFactorGraph& fg) {
  //   auto prior = boost::make_shared<DiscreteDistribution>(modes[1], "1/1");
  //   fg.push_discrete(prior);
  //   for (size_t k = 1; k < K - 1; k++) {
  //     auto parents = {modes[k]};
  //     auto conditional = boost::make_shared<DiscreteConditional>(
  //         modes[k + 1], parents, "1/2 3/2");
  //     fg.push_discrete(conditional);
  //   }
  // }

  // Add "mode chain" to GaussianHybridFactorGraph
  HybridGaussianFactorGraph addModeChain(HybridGaussianFactorGraph& fg) {
    auto prior = boost::make_shared<DiscreteDistribution>(modes[1], "1/1");
    fg.push_discrete(prior);
    for (size_t k = 1; k < K - 1; k++) {
      auto parents = {modes[k]};
      auto conditional = boost::make_shared<DiscreteConditional>(
          modes[k + 1], parents, "1/2 3/2");
      fg.push_discrete(conditional);
    }
    return fg;
  }
};

/**
 * @brief Create a switching system chain. A switching system is a continuous
 * system which depends on a discrete mode at each time step of the chain.
 *
 * @param n The number of chain elements.
 * @param keyFunc The functional to help specify the continuous key.
 * @param dKeyFunc The functional to help specify the discrete key.
 * @return HybridGaussianFactorGraph::shared_ptr
 */
inline HybridGaussianFactorGraph::shared_ptr makeSwitchingChain(
    size_t n, std::function<Key(int)> keyFunc = X,
    std::function<Key(int)> dKeyFunc = C) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(keyFunc(1), I_3x3, Z_3x1));

  // keyFunc(1) to keyFunc(n+1)
  for (size_t t = 1; t < n; t++) {
    hfg.add(GaussianMixtureFactor::FromFactors(
        {keyFunc(t), keyFunc(t + 1)}, {{dKeyFunc(t), 2}},
        {boost::make_shared<JacobianFactor>(keyFunc(t), I_3x3, keyFunc(t + 1),
                                            I_3x3, Z_3x1),
         boost::make_shared<JacobianFactor>(keyFunc(t), I_3x3, keyFunc(t + 1),
                                            I_3x3, Vector3::Ones())}));

    if (t > 1) {
      hfg.add(DecisionTreeFactor({{dKeyFunc(t - 1), 2}, {dKeyFunc(t), 2}},
                                 "0 1 1 3"));
    }
  }

  return boost::make_shared<HybridGaussianFactorGraph>(std::move(hfg));
}

/**
 * @brief
 *
 * @param input The original ordering.
 * @return std::pair<KeyVector, std::vector<int>>
 */
inline std::pair<KeyVector, std::vector<int>> makeBinaryOrdering(
    std::vector<Key>& input) {
  KeyVector new_order;

  std::vector<int> levels(input.size());
  std::function<void(std::vector<Key>::iterator, std::vector<Key>::iterator,
                     int)>
      bsg = [&bsg, &new_order, &levels, &input](
                std::vector<Key>::iterator begin,
                std::vector<Key>::iterator end, int lvl) {
        if (std::distance(begin, end) > 1) {
          std::vector<Key>::iterator pivot =
              begin + std::distance(begin, end) / 2;

          new_order.push_back(*pivot);
          levels[std::distance(input.begin(), pivot)] = lvl;
          bsg(begin, pivot, lvl + 1);
          bsg(pivot + 1, end, lvl + 1);
        } else if (std::distance(begin, end) == 1) {
          new_order.push_back(*begin);
          levels[std::distance(input.begin(), begin)] = lvl;
        }
      };

  bsg(input.begin(), input.end(), 0);
  std::reverse(new_order.begin(), new_order.end());

  return {new_order, levels};
}

}  // namespace gtsam
