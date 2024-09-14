/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridGaussianConditional.cpp
 * @brief   Unit tests for HybridGaussianConditional class
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>

#include <vector>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

// Common constants
static const Key modeKey = M(0);
static const DiscreteKey mode(modeKey, 2);
static const VectorValues vv{{Z(0), Vector1(4.9)}, {X(0), Vector1(5.0)}};
static const DiscreteValues assignment0{{M(0), 0}}, assignment1{{M(0), 1}};
static const HybridValues hv0{vv, assignment0};
static const HybridValues hv1{vv, assignment1};

/* ************************************************************************* */
namespace equal_constants {
// Create a simple HybridGaussianConditional
const double commonSigma = 2.0;
const std::vector<GaussianConditional::shared_ptr> conditionals{
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             commonSigma),
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             commonSigma)};
const HybridGaussianConditional mixture({Z(0)}, {X(0)}, {mode}, conditionals);
}  // namespace equal_constants

/* ************************************************************************* */
/// Check that invariants hold
TEST(HybridGaussianConditional, Invariants) {
  using namespace equal_constants;

  // Check that the mixture normalization constant is the max of all constants
  // which are all equal, in this case, hence:
  const double K = mixture.logNormalizationConstant();
  EXPECT_DOUBLES_EQUAL(K, conditionals[0]->logNormalizationConstant(), 1e-8);
  EXPECT_DOUBLES_EQUAL(K, conditionals[1]->logNormalizationConstant(), 1e-8);

  EXPECT(HybridGaussianConditional::CheckInvariants(mixture, hv0));
  EXPECT(HybridGaussianConditional::CheckInvariants(mixture, hv1));
}

/* ************************************************************************* */
/// Check LogProbability.
TEST(HybridGaussianConditional, LogProbability) {
  using namespace equal_constants;
  auto actual = mixture.logProbability(vv);

  // Check result.
  std::vector<DiscreteKey> discrete_keys = {mode};
  std::vector<double> leaves = {conditionals[0]->logProbability(vv),
                                conditionals[1]->logProbability(vv)};
  AlgebraicDecisionTree<Key> expected(discrete_keys, leaves);

  EXPECT(assert_equal(expected, actual, 1e-6));

  // Check for non-tree version.
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    EXPECT_DOUBLES_EQUAL(conditionals[mode]->logProbability(vv),
                         mixture.logProbability(hv), 1e-8);
  }
}

/* ************************************************************************* */
/// Check error.
TEST(HybridGaussianConditional, Error) {
  using namespace equal_constants;
  auto actual = mixture.errorTree(vv);

  // Check result.
  std::vector<DiscreteKey> discrete_keys = {mode};
  std::vector<double> leaves = {conditionals[0]->error(vv),
                                conditionals[1]->error(vv)};
  AlgebraicDecisionTree<Key> expected(discrete_keys, leaves);

  EXPECT(assert_equal(expected, actual, 1e-6));

  // Check for non-tree version.
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    EXPECT_DOUBLES_EQUAL(conditionals[mode]->error(vv), mixture.error(hv),
                         1e-8);
  }
}

/* ************************************************************************* */
/// Check that the likelihood is proportional to the conditional density given
/// the measurements.
TEST(HybridGaussianConditional, Likelihood) {
  using namespace equal_constants;

  // Compute likelihood
  auto likelihood = mixture.likelihood(vv);

  // Check that the mixture error and the likelihood error are the same.
  EXPECT_DOUBLES_EQUAL(mixture.error(hv0), likelihood->error(hv0), 1e-8);
  EXPECT_DOUBLES_EQUAL(mixture.error(hv1), likelihood->error(hv1), 1e-8);

  // Check that likelihood error is as expected, i.e., just the errors of the
  // individual likelihoods, in the `equal_constants` case.
  std::vector<DiscreteKey> discrete_keys = {mode};
  std::vector<double> leaves = {conditionals[0]->likelihood(vv)->error(vv),
                                conditionals[1]->likelihood(vv)->error(vv)};
  AlgebraicDecisionTree<Key> expected(discrete_keys, leaves);
  EXPECT(assert_equal(expected, likelihood->errorTree(vv), 1e-6));

  // Check that the ratio of probPrime to evaluate is the same for all modes.
  std::vector<double> ratio(2);
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    ratio[mode] = std::exp(-likelihood->error(hv)) / mixture.evaluate(hv);
  }
  EXPECT_DOUBLES_EQUAL(ratio[0], ratio[1], 1e-8);
}

/* ************************************************************************* */
namespace mode_dependent_constants {
// Create a HybridGaussianConditional with mode-dependent noise models.
// 0 is low-noise, 1 is high-noise.
const std::vector<GaussianConditional::shared_ptr> conditionals{
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             0.5),
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             3.0)};
const HybridGaussianConditional mixture({Z(0)}, {X(0)}, {mode}, conditionals);
}  // namespace mode_dependent_constants

/* ************************************************************************* */
// Create a test for continuousParents.
TEST(HybridGaussianConditional, ContinuousParents) {
  using namespace mode_dependent_constants;
  const KeyVector continuousParentKeys = mixture.continuousParents();
  // Check that the continuous parent keys are correct:
  EXPECT(continuousParentKeys.size() == 1);
  EXPECT(continuousParentKeys[0] == X(0));
}

/* ************************************************************************* */
/// Check that the likelihood is proportional to the conditional density given
/// the measurements.
TEST(HybridGaussianConditional, Likelihood2) {
  using namespace mode_dependent_constants;

  // Compute likelihood
  auto likelihood = mixture.likelihood(vv);

  // Check that the mixture error and the likelihood error are as expected,
  // this invariant is the same as the equal noise case:
  EXPECT_DOUBLES_EQUAL(mixture.error(hv0), likelihood->error(hv0), 1e-8);
  EXPECT_DOUBLES_EQUAL(mixture.error(hv1), likelihood->error(hv1), 1e-8);

  // Check the detailed JacobianFactor calculation for mode==1.
  {
    // We have a JacobianFactor
    const auto gf1 = (*likelihood)(assignment1).first;
    const auto jf1 = std::dynamic_pointer_cast<JacobianFactor>(gf1);
    CHECK(jf1);

    // It has 2 rows, not 1!
    CHECK(jf1->rows() == 2);

    // Check that the constant C1 is properly encoded in the JacobianFactor.
    const double C1 = mixture.logNormalizationConstant() -
                      conditionals[1]->logNormalizationConstant();
    const double c1 = std::sqrt(2.0 * C1);
    Vector expected_unwhitened(2);
    expected_unwhitened << 4.9 - 5.0, -c1;
    Vector actual_unwhitened = jf1->unweighted_error(vv);
    EXPECT(assert_equal(expected_unwhitened, actual_unwhitened));

    // Make sure the noise model does not touch it.
    Vector expected_whitened(2);
    expected_whitened << (4.9 - 5.0) / 3.0, -c1;
    Vector actual_whitened = jf1->error_vector(vv);
    EXPECT(assert_equal(expected_whitened, actual_whitened));

    // Check that the error is equal to the mixture error:
    EXPECT_DOUBLES_EQUAL(mixture.error(hv1), jf1->error(hv1), 1e-8);
  }

  // Check that the ratio of probPrime to evaluate is the same for all modes.
  std::vector<double> ratio(2);
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    ratio[mode] = std::exp(-likelihood->error(hv)) / mixture.evaluate(hv);
  }
  EXPECT_DOUBLES_EQUAL(ratio[0], ratio[1], 1e-8);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
