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
 */
HybridBayesNet createHybridBayesNet(int num_measurements = 1) {
  HybridBayesNet bayesNet;

  // Create Gaussian mixture z_i = x0 + noise for each measurement.
  for (int i = 0; i < num_measurements; i++) {
    const auto conditional0 = boost::make_shared<GaussianConditional>(
        GaussianConditional::FromMeanAndStddev(Z(i), I_1x1, X(0), Z_1x1, 0.5));
    const auto conditional1 = boost::make_shared<GaussianConditional>(
        GaussianConditional::FromMeanAndStddev(Z(i), I_1x1, X(0), Z_1x1, 3));
    GaussianMixture gm({Z(i)}, {X(0)}, {mode}, {conditional0, conditional1});
    bayesNet.emplaceMixture(gm);  // copy :-(
  }

  // Create prior on X(0).
  const auto prior_on_x0 =
      GaussianConditional::FromMeanAndStddev(X(0), Vector1(5.0), 0.5);
  bayesNet.emplaceGaussian(prior_on_x0);  // copy :-(

  // Add prior on mode.
  bayesNet.emplaceDiscrete(mode, "4/6");

  return bayesNet;
}

/**
 * Convert a hybrid Bayes net to a hybrid Gaussian factor graph.
 */
HybridGaussianFactorGraph convertBayesNet(const HybridBayesNet& bayesNet,
                                          const VectorValues& measurements) {
  HybridGaussianFactorGraph fg;
  int num_measurements = bayesNet.size() - 2;
  for (int i = 0; i < num_measurements; i++) {
    auto conditional = bayesNet.atMixture(i);
    auto factor = conditional->likelihood({{Z(i), measurements.at(Z(i))}});
    fg.push_back(factor);
  }
  fg.push_back(bayesNet.atGaussian(num_measurements));
  fg.push_back(bayesNet.atDiscrete(num_measurements + 1));
  return fg;
}

/**
 * Create a tiny two variable hybrid factor graph which represents a discrete
 * mode and a continuous variable x0, given a number of measurements of the
 * continuous variable x0. If no measurements are given, they are sampled from
 * the generative Bayes net model HybridBayesNet::Example(num_measurements)
 */
HybridGaussianFactorGraph createHybridGaussianFactorGraph(
    int num_measurements = 1,
    boost::optional<VectorValues> measurements = boost::none) {
  auto bayesNet = createHybridBayesNet(num_measurements);
  if (measurements) {
    return convertBayesNet(bayesNet, *measurements);
  } else {
    return convertBayesNet(bayesNet, bayesNet.sample().continuous());
  }
}

}  // namespace tiny
}  // namespace gtsam
