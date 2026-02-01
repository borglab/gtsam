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

#include "Switching.h"
namespace gtsam {
using symbol_shorthand::D;
using symbol_shorthand::X;

namespace discrete_mixture_fixture {
// We'll make a variable with 2 possible assignments
[[maybe_unused]] static DiscreteKey dk(D(1), 2);

// Key for single continuous variable
[[maybe_unused]] static Key x1 = X(1);

[[maybe_unused]] static std::shared_ptr<noiseModel::Isotropic> priorNoise1() {
  return noiseModel::Isotropic::Sigma(1, 1.0);
}

[[maybe_unused]] static std::shared_ptr<PriorFactor<double>> f1() {
  return std::make_shared<PriorFactor<double>>(X(1), 0.0, priorNoise1());
}

[[maybe_unused]] static std::shared_ptr<noiseModel::Isotropic>
priorNoiseNullHypo() {
  return noiseModel::Isotropic::Sigma(1, 8.0);
}

[[maybe_unused]] static std::shared_ptr<PriorFactor<double>> fNullHypo() {
  return std::make_shared<PriorFactor<double>>(X(1), 0.0, priorNoiseNullHypo());
}

}  // namespace discrete_mixture_fixture

namespace two_component_fixture {
[[maybe_unused]] static std::vector<GaussianFactor::shared_ptr> components(
    Key key) {
  return {std::make_shared<JacobianFactor>(key, I_3x3, Z_3x1),
          std::make_shared<JacobianFactor>(key, I_3x3, Vector3::Ones())};
}
}  // namespace two_component_fixture

namespace estimation_fixture {
[[maybe_unused]] static std::vector<double> measurements = {
    0, 1, 2, 2, 2, 2, 3, 4, 5, 6, 6, 7, 8, 9, 9, 9, 10, 11, 11, 11, 11};

// Ground truth discrete seq
[[maybe_unused]] static std::vector<size_t> discrete_seq = {
    1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0};

[[maybe_unused]] static Switching InitializeEstimationProblem(
    const size_t K, const double between_sigma, const double measurement_sigma,
    const std::vector<double>& measurements,
    const std::string& transitionProbabilityTable,
    HybridNonlinearFactorGraph* graph, Values* initial) {
  Switching switching(K, between_sigma, measurement_sigma, measurements,
                      transitionProbabilityTable);

  // Add prior on M(0)
  graph->push_back(switching.modeChain.at(0));

  // Add the X(0) prior
  graph->push_back(switching.unaryFactors.at(0));
  initial->insert(X(0), switching.linearizationPoint.at<double>(X(0)));

  return switching;
}

}  // namespace estimation_fixture
}  // namespace gtsam
