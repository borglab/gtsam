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
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <vector>

#pragma once

namespace gtsam {

using symbol_shorthand::C;
using symbol_shorthand::M;
using symbol_shorthand::X;

/**
 * @brief Create a switching system chain. A switching system is a continuous
 * system which depends on a discrete mode at each time step of the chain.
 *
 * @param K The number of chain elements.
 * @param x The functional to help specify the continuous key.
 * @param m The functional to help specify the discrete key.
 * @return HybridGaussianFactorGraph::shared_ptr
 */
inline HybridGaussianFactorGraph::shared_ptr makeSwitchingChain(
    size_t K, std::function<Key(int)> x = X, std::function<Key(int)> m = M) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(x(1), I_3x3, Z_3x1));

  // x(1) to x(n+1)
  for (size_t k = 1; k < K; k++) {
    DiscreteKeys dKeys{{m(k), 2}};
    std::vector<GaussianFactor::shared_ptr> components;
    components.emplace_back(
        new JacobianFactor(x(k), I_3x3, x(k + 1), I_3x3, Z_3x1));
    components.emplace_back(
        new JacobianFactor(x(k), I_3x3, x(k + 1), I_3x3, Vector3::Ones()));
    hfg.add(HybridGaussianFactor({m(k), 2}, components));

    if (k > 1) {
      hfg.add(DecisionTreeFactor({{m(k - 1), 2}, {m(k), 2}}, "0 1 1 3"));
    }
  }

  return std::make_shared<HybridGaussianFactorGraph>(std::move(hfg));
}

/**
 * @brief Return the ordering as a binary tree such that all parent nodes are
 * above their children.
 *
 * This will result in a nested dissection Bayes tree after elimination.
 *
 * @param input The original ordering.
 * @return std::pair<KeyVector, std::vector<int>>
 */
inline std::pair<KeyVector, std::vector<int>> makeBinaryOrdering(
    std::vector<Key> &input) {
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

/* ****************************************************************************/
using MotionModel = BetweenFactor<double>;

// Test fixture with switching network.
/// ϕ(X(0)) .. ϕ(X(k),X(k+1)) .. ϕ(X(k);z_k) .. ϕ(M(0)) .. ϕ(M(K-3),M(K-2))
struct Switching {
  size_t K;
  DiscreteKeys modes;
  HybridNonlinearFactorGraph nonlinearFactorGraph;
  HybridGaussianFactorGraph linearizedFactorGraph;
  Values linearizationPoint;

  /**
   * @brief Create with given number of time steps.
   *
   * @param K The total number of timesteps.
   * @param between_sigma The stddev between poses.
   * @param prior_sigma The stddev on priors (also used for measurements).
   * @param measurements Vector of measurements for each timestep.
   */
  Switching(size_t K, double between_sigma = 1.0, double prior_sigma = 0.1,
            std::vector<double> measurements = {},
            std::string discrete_transition_prob = "1/2 3/2")
      : K(K) {
    using noiseModel::Isotropic;

    // Create DiscreteKeys for K-1 binary modes.
    for (size_t k = 0; k < K - 1; k++) {
      modes.emplace_back(M(k), 2);
    }

    // If measurements are not provided, we just have the robot moving forward.
    if (measurements.size() == 0) {
      for (size_t k = 0; k < K; k++) {
        measurements.push_back(k);
      }
    }

    // Create hybrid factor graph.

    // Add a prior ϕ(X(0)) on X(0).
    nonlinearFactorGraph.emplace_shared<PriorFactor<double>>(
        X(0), measurements.at(0), Isotropic::Sigma(1, prior_sigma));

    // Add "motion models" ϕ(X(k),X(k+1)).
    for (size_t k = 0; k < K - 1; k++) {
      auto motion_models = motionModels(k, between_sigma);
      nonlinearFactorGraph.emplace_shared<HybridNonlinearFactor>(modes[k],
                                                                 motion_models);
    }

    // Add measurement factors ϕ(X(k);z_k).
    auto measurement_noise = Isotropic::Sigma(1, prior_sigma);
    for (size_t k = 1; k < K; k++) {
      nonlinearFactorGraph.emplace_shared<PriorFactor<double>>(
          X(k), measurements.at(k), measurement_noise);
    }

    // Add "mode chain" ϕ(M(0)) ϕ(M(0),M(1)) ... ϕ(M(K-3),M(K-2))
    addModeChain(&nonlinearFactorGraph, discrete_transition_prob);

    // Create the linearization point.
    for (size_t k = 0; k < K; k++) {
      linearizationPoint.insert<double>(X(k), static_cast<double>(k + 1));
    }

    linearizedFactorGraph = *nonlinearFactorGraph.linearize(linearizationPoint);
  }

  // Create motion models for a given time step
  static std::vector<NoiseModelFactor::shared_ptr> motionModels(
      size_t k, double sigma = 1.0) {
    auto noise_model = noiseModel::Isotropic::Sigma(1, sigma);
    auto still =
             std::make_shared<MotionModel>(X(k), X(k + 1), 0.0, noise_model),
         moving =
             std::make_shared<MotionModel>(X(k), X(k + 1), 1.0, noise_model);
    return {still, moving};
  }

  /**
   * @brief Add "mode chain" to HybridNonlinearFactorGraph from M(0) to M(K-2).
   * E.g. if K=4, we want M0, M1 and M2.
   *
   * @param fg The factor graph to which the mode chain is added.
   */
  template <typename FACTORGRAPH>
  void addModeChain(FACTORGRAPH *fg,
                    std::string discrete_transition_prob = "1/2 3/2") {
    fg->template emplace_shared<DiscreteDistribution>(modes[0], "1/1");
    for (size_t k = 0; k < K - 2; k++) {
      auto parents = {modes[k]};
      fg->template emplace_shared<DiscreteConditional>(
          modes[k + 1], parents, discrete_transition_prob);
    }
  }
};

}  // namespace gtsam
