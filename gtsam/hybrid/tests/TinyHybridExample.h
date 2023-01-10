/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2023, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file TinyHybridExample.h
 *  @date December, 2022
 *  @author Frank Dellaert
 */

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#pragma once

namespace gtsam {
namespace tiny {

using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

// Create mode key: 0 is low-noise, 1 is high-noise.
const DiscreteKey mode{M(0), 2};

/**
 * Create a tiny two variable hybrid model which represents
 * the generative probability P(z,x,mode) = P(z|x,mode)P(x)P(mode).
 * num_measurements is the number of measurements of the continuous variable x0.
 * If manyModes is true, then we introduce one mode per measurement.
 */
inline HybridBayesNet createHybridBayesNet(int num_measurements = 1,
                                           bool manyModes = false) {
  HybridBayesNet bayesNet;

  // Create Gaussian mixture z_i = x0 + noise for each measurement.
  for (int i = 0; i < num_measurements; i++) {
    const auto mode_i = manyModes ? DiscreteKey{M(i), 2} : mode;
    bayesNet.emplace_back(
        new GaussianMixture({Z(i)}, {X(0)}, {mode_i},
                            {GaussianConditional::sharedMeanAndStddev(
                                 Z(i), I_1x1, X(0), Z_1x1, 0.5),
                             GaussianConditional::sharedMeanAndStddev(
                                 Z(i), I_1x1, X(0), Z_1x1, 3)}));
  }

  // Create prior on X(0).
  bayesNet.push_back(
      GaussianConditional::sharedMeanAndStddev(X(0), Vector1(5.0), 0.5));

  // Add prior on mode.
  const size_t nrModes = manyModes ? num_measurements : 1;
  for (const size_t i = 0; i < nrModes; i++) {
    bayesNet.emplace_back(new DiscreteConditional({M(i), 2}, "4/6"));
  }
  return bayesNet;
}

/**
 * Create a tiny two variable hybrid factor graph which represents a discrete
 * mode and a continuous variable x0, given a number of measurements of the
 * continuous variable x0. If no measurements are given, they are sampled from
 * the generative Bayes net model HybridBayesNet::Example(num_measurements)
 */
inline HybridGaussianFactorGraph createHybridGaussianFactorGraph(
    int num_measurements = 1,
    boost::optional<VectorValues> measurements = boost::none,
    bool manyModes = false) {
  auto bayesNet = createHybridBayesNet(num_measurements, manyModes);
  if (measurements) {
    // Use the measurements to create a hybrid factor graph.
    return bayesNet.toFactorGraph(*measurements);
  } else {
    // Sample from the generative model to create a hybrid factor graph.
    return bayesNet.toFactorGraph(bayesNet.sample().continuous());
  }
}

}  // namespace tiny
}  // namespace gtsam
