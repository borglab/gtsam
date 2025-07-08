/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file DiscreteFixture.h
 *  @date May 26, 2025
 *  @author Varun Agrawal
 */

#include <gtsam/discrete/DecisionTreeFactor.h>

using namespace gtsam;
using symbol_shorthand::D;
using symbol_shorthand::X;

namespace discrete_mixture_fixture {
// We'll make a variable with 2 possible assignments
DiscreteKey dk(D(1), 2);

// Key for single continuous variable
Key x1 = X(1);

// Make a factor for non-null hypothesis
inline std::shared_ptr<gtsam::PriorFactor<double>> f1() {
  const double loc = 0.0;
  const double sigma1 = 1.0;
  static auto prior_noise1 = noiseModel::Isotropic::Sigma(1, sigma1);
  static auto f = std::make_shared<PriorFactor<double>>(x1, loc, prior_noise1);
  return f;
}

// Make a factor for null hypothesis
inline std::shared_ptr<gtsam::PriorFactor<double>> fNullHypo() {
  const double loc = 0.0;
  const double sigmaNullHypo = 8.0;
  static auto prior_noiseNullHypo = noiseModel::Isotropic::Sigma(1, sigmaNullHypo);
  static auto f = std::make_shared<PriorFactor<double>>(x1, loc, prior_noiseNullHypo);
  return f;
}

inline std::shared_ptr<gtsam::noiseModel::Isotropic> prior_noise1() {
  const double sigma1 = 1.0;
  static auto prior_noise1 = noiseModel::Isotropic::Sigma(1, sigma1);
  return prior_noise1;
}

inline std::shared_ptr<gtsam::noiseModel::Isotropic> prior_noiseNullHypo() {
  const double sigmaNullHypo = 8.0;
  static auto prior_noiseNullHypo = noiseModel::Isotropic::Sigma(1, sigmaNullHypo);
  return prior_noiseNullHypo;
}
}  // namespace discrete_mixture_fixture
