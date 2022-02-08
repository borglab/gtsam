/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Switching.h
 * @brief   Test harness for hybrid factor graph.
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/hybrid/DCMixtureFactor.h>
#include <gtsam/hybrid/NonlinearHybridFactorGraph.h>
#include <gtsam/hybrid/GaussianHybridFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {
using MotionModel = BetweenFactor<double>;

/* ****************************************************************************/
// Test fixture with switching network.
using MotionMixture = DCMixtureFactor<MotionModel>;
struct Switching {
  size_t K;
  DiscreteKeys modes;
  NonlinearHybridFactorGraph nonlinearFactorGraph;
  GaussianHybridFactorGraph linearizedFactorGraph;
  Values linearizationPoint;

  /// Create with given number of time steps.
  Switching(size_t K, double between_sigma = 1.0, double prior_sigma = 0.1)
      : K(K) {
    using symbol_shorthand::M;
    using symbol_shorthand::X;

    // Create DiscreteKeys for binary K modes, modes[0] will not be used.
    for (size_t k = 0; k <= K; k++) {
      modes.emplace_back(M(k), 2);
    }

    // Create hybrid factor graph.
    // Add a prior on X(1).
    auto prior = boost::make_shared < PriorFactor < double >> (
        X(1), 0, noiseModel::Isotropic::Sigma(1, prior_sigma));
    nonlinearFactorGraph.push_nonlinear(prior);

    // Add "motion models".
    for (size_t k = 1; k < K; k++) {
      auto keys = {X(k), X(k + 1)};
      auto components = motionModels(k);
      nonlinearFactorGraph.emplace_dc<MotionMixture>(
          keys, DiscreteKeys{modes[k]}, components);
    }

    // Add measurement factors
    auto measurement_noise = noiseModel::Isotropic::Sigma(1, 0.1);
    for (size_t k = 1; k <= K; k++) {
      nonlinearFactorGraph.emplace_nonlinear < PriorFactor < double >> (
          X(k), 1.0 * (k - 1), measurement_noise);
    }

    // Add "mode chain"
    addModeChain(&nonlinearFactorGraph);

    // Create the linearization point.
    for (size_t k = 1; k <= K; k++) {
      linearizationPoint.insert<double>(X(k), static_cast<double>(k));
    }

    linearizedFactorGraph = nonlinearFactorGraph.linearize(linearizationPoint);
  }

  // Create motion models for a given time step
  static std::vector <MotionModel::shared_ptr> motionModels(size_t k,
                                                            double sigma = 1.0) {
    using symbol_shorthand::M;
    using symbol_shorthand::X;

    auto noise_model = noiseModel::Isotropic::Sigma(1, sigma);
    auto still =
        boost::make_shared<MotionModel>(X(k), X(k + 1), 0.0, noise_model),
        moving =
        boost::make_shared<MotionModel>(X(k), X(k + 1), 1.0, noise_model);
    return {still, moving};
  }

  // Add "mode chain" to NonlinearHybridFactorGraph
  void addModeChain(NonlinearHybridFactorGraph *fg) {
    auto prior = boost::make_shared<DiscreteDistribution>(modes[1], "1/1");
    fg->push_discrete(prior);
    for (size_t k = 1; k < K - 1; k++) {
      auto parents = {modes[k]};
      auto conditional = boost::make_shared<DiscreteConditional>(
          modes[k + 1], parents, "1/2 3/2");
      fg->push_discrete(conditional);
    }
  }

  // Add "mode chain" to GaussianHybridFactorGraph
  void addModeChain(GaussianHybridFactorGraph *fg) {
    auto prior = boost::make_shared<DiscreteDistribution>(modes[1], "1/1");
    fg->push_discrete(prior);
    for (size_t k = 1; k < K - 1; k++) {
      auto parents = {modes[k]};
      auto conditional = boost::make_shared<DiscreteConditional>(
          modes[k + 1], parents, "1/2 3/2");
      fg->push_discrete(conditional);
    }
  }
};

}
