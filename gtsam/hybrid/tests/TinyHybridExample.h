/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file TinyHybrdiExample.h
 *  @date Mar 11, 2022
 *  @author Varun Agrawal
 *  @author Fan Jiang
 */

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/inference/Symbol.h>
#pragma once

namespace gtsam {
namespace tiny {

using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

/**
 * Create a tiny two variable hybrid model which represents
 * the generative probability P(z, x, n) = P(z | x, n)P(x)P(n).
 */
static HybridBayesNet createHybridBayesNet(int num_measurements = 1) {
  // Create hybrid Bayes net.
  HybridBayesNet bayesNet;

  // Create mode key: 0 is low-noise, 1 is high-noise.
  const DiscreteKey mode{M(0), 2};

  // Create Gaussian mixture Z(0) = X(0) + noise for each measurement.
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
      GaussianConditional::FromMeanAndStddev(X(0), Vector1(5.0), 5.0);
  bayesNet.emplaceGaussian(prior_on_x0);  // copy :-(

  // Add prior on mode.
  bayesNet.emplaceDiscrete(mode, "4/6");

  return bayesNet;
}

}  // namespace tiny
}  // namespace gtsam
