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
 * numMeasurements is the number of measurements of the continuous variable x0.
 * If manyModes is true, then we introduce one mode per measurement.
 */
inline HybridBayesNet createHybridBayesNet(int numMeasurements = 1,
                                    bool manyModes = false) {
  HybridBayesNet bayesNet;

  // Create Gaussian mixture z_i = x0 + noise for each measurement.
  for (int i = 0; i < numMeasurements; i++) {
    const auto conditional0 = boost::make_shared<GaussianConditional>(
        GaussianConditional::FromMeanAndStddev(Z(i), I_1x1, X(0), Z_1x1, 0.5));
    const auto conditional1 = boost::make_shared<GaussianConditional>(
        GaussianConditional::FromMeanAndStddev(Z(i), I_1x1, X(0), Z_1x1, 3));
    const auto mode_i = manyModes ? DiscreteKey{M(i), 2} : mode;
    GaussianMixture gm({Z(i)}, {X(0)}, {mode_i}, {conditional0, conditional1});
    bayesNet.emplaceMixture(gm);  // copy :-(
  }

  // Create prior on X(0).
  const auto prior_on_x0 =
      GaussianConditional::FromMeanAndStddev(X(0), Vector1(5.0), 0.5);
  bayesNet.emplaceGaussian(prior_on_x0);  // copy :-(

  // Add prior on mode.
  const size_t nrModes = manyModes ? numMeasurements : 1;
  for (int i = 0; i < nrModes; i++) {
    bayesNet.emplaceDiscrete(DiscreteKey{M(i), 2}, "4/6");
  }
  return bayesNet;
}

/**
 * Convert a hybrid Bayes net to a hybrid Gaussian factor graph.
 */
inline HybridGaussianFactorGraph convertBayesNet(
    const HybridBayesNet& bayesNet, const VectorValues& measurements) {
  HybridGaussianFactorGraph fg;
  // For all nodes in the Bayes net, if its frontal variable is in measurements,
  // replace it by a likelihood factor:
  for (const HybridConditional::shared_ptr& conditional : bayesNet) {
    if (measurements.exists(conditional->firstFrontalKey())) {
      if (auto gc = conditional->asGaussian())
        fg.push_back(gc->likelihood(measurements));
      else if (auto gm = conditional->asMixture())
        fg.push_back(gm->likelihood(measurements));
      else {
        throw std::runtime_error("Unknown conditional type");
      }
    } else {
      fg.push_back(conditional);
    }
  }
  return fg;
}

/**
 * Create a tiny two variable hybrid factor graph which represents a discrete
 * mode and a continuous variable x0, given a number of measurements of the
 * continuous variable x0. If no measurements are given, they are sampled from
 * the generative Bayes net model HybridBayesNet::Example(numMeasurements)
 */
inline HybridGaussianFactorGraph createHybridGaussianFactorGraph(
    int numMeasurements = 1,
    boost::optional<VectorValues> measurements = boost::none,
    bool manyModes = false) {
  auto bayesNet = createHybridBayesNet(numMeasurements, manyModes);
  if (measurements) {
    // Use the measurements to create a hybrid factor graph.
    return convertBayesNet(bayesNet, *measurements);
  } else {
    // Sample from the generative model to create a hybrid factor graph.
    return convertBayesNet(bayesNet, bayesNet.sample().continuous());
  }
}

}  // namespace tiny
}  // namespace gtsam
