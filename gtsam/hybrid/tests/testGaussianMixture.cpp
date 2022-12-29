/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianMixture.cpp
 * @brief   Unit tests for GaussianMixture class
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>

#include <vector>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

/* ************************************************************************* */
/* Check construction of GaussianMixture P(x1 | x2, m1) as well as accessing a
 * specific mode i.e. P(x1 | x2, m1=1).
 */
TEST(GaussianMixture, Equals) {
  // create a conditional gaussian node
  Matrix S1(2, 2);
  S1(0, 0) = 1;
  S1(1, 0) = 2;
  S1(0, 1) = 3;
  S1(1, 1) = 4;

  Matrix S2(2, 2);
  S2(0, 0) = 6;
  S2(1, 0) = 0.2;
  S2(0, 1) = 8;
  S2(1, 1) = 0.4;

  Matrix R1(2, 2);
  R1(0, 0) = 0.1;
  R1(1, 0) = 0.3;
  R1(0, 1) = 0.0;
  R1(1, 1) = 0.34;

  Matrix R2(2, 2);
  R2(0, 0) = 0.1;
  R2(1, 0) = 0.3;
  R2(0, 1) = 0.0;
  R2(1, 1) = 0.34;

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  Vector2 d1(0.2, 0.5), d2(0.5, 0.2);

  auto conditional0 = boost::make_shared<GaussianConditional>(X(1), d1, R1,
                                                              X(2), S1, model),
       conditional1 = boost::make_shared<GaussianConditional>(X(1), d2, R2,
                                                              X(2), S2, model);

  // Create decision tree
  DiscreteKey m1(1, 2);
  GaussianMixture::Conditionals conditionals(
      {m1},
      vector<GaussianConditional::shared_ptr>{conditional0, conditional1});
  GaussianMixture mixture({X(1)}, {X(2)}, {m1}, conditionals);

  // Let's check that this worked:
  DiscreteValues mode;
  mode[m1.first] = 1;
  auto actual = mixture(mode);
  EXPECT(actual == conditional1);
}

/* ************************************************************************* */
/// Test error method of GaussianMixture.
TEST(GaussianMixture, Error) {
  Matrix22 S1 = Matrix22::Identity();
  Matrix22 S2 = Matrix22::Identity() * 2;
  Matrix22 R1 = Matrix22::Ones();
  Matrix22 R2 = Matrix22::Ones();
  Vector2 d1(1, 2), d2(2, 1);

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  auto conditional0 = boost::make_shared<GaussianConditional>(X(1), d1, R1,
                                                              X(2), S1, model),
       conditional1 = boost::make_shared<GaussianConditional>(X(1), d2, R2,
                                                              X(2), S2, model);

  // Create decision tree
  DiscreteKey m1(M(1), 2);
  GaussianMixture::Conditionals conditionals(
      {m1},
      vector<GaussianConditional::shared_ptr>{conditional0, conditional1});
  GaussianMixture mixture({X(1)}, {X(2)}, {m1}, conditionals);

  VectorValues values;
  values.insert(X(1), Vector2::Ones());
  values.insert(X(2), Vector2::Zero());
  auto error_tree = mixture.error(values);

  // regression
  std::vector<DiscreteKey> discrete_keys = {m1};
  std::vector<double> leaves = {0.5, 4.3252595};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, leaves);

  EXPECT(assert_equal(expected_error, error_tree, 1e-6));

  // Regression for non-tree version.
  DiscreteValues assignment;
  assignment[M(1)] = 0;
  EXPECT_DOUBLES_EQUAL(0.5, mixture.error(values, assignment), 1e-8);
  assignment[M(1)] = 1;
  EXPECT_DOUBLES_EQUAL(4.3252595155709335, mixture.error(values, assignment),
                       1e-8);
}

/* ************************************************************************* */
// Create mode key: 0 is low-noise, 1 is high-noise.
static const Key modeKey = M(0);
static const DiscreteKey mode(modeKey, 2);

// Create a simple GaussianMixture
static GaussianMixture createSimpleGaussianMixture() {
  // Create Gaussian mixture Z(0) = X(0) + noise.
  // TODO(dellaert): making copies below is not ideal !
  Matrix1 I = Matrix1::Identity();
  const auto conditional0 = boost::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 0.5));
  const auto conditional1 = boost::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 3));
  const auto gm = GaussianMixture::FromConditionals(
      {Z(0)}, {X(0)}, {mode}, {conditional0, conditional1});
  return gm;
}

/* ************************************************************************* */
std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys& dkeys) {
  std::set<DiscreteKey> s;
  s.insert(dkeys.begin(), dkeys.end());
  return s;
}

// Get only the continuous parent keys as a KeyVector:
KeyVector continuousParents(const GaussianMixture& gm) {
  // Get all parent keys:
  const auto range = gm.parents();
  KeyVector continuousParentKeys(range.begin(), range.end());
  // Loop over all discrete keys:
  for (const auto& discreteKey : gm.discreteKeys()) {
    const Key key = discreteKey.first;
    // remove that key from continuousParentKeys:
    continuousParentKeys.erase(std::remove(continuousParentKeys.begin(),
                                           continuousParentKeys.end(), key),
                               continuousParentKeys.end());
  }
  return continuousParentKeys;
}

// Create a test for continuousParents.
TEST(GaussianMixture, ContinuousParents) {
  const GaussianMixture gm = createSimpleGaussianMixture();
  const KeyVector continuousParentKeys = continuousParents(gm);
  // Check that the continuous parent keys are correct:
  EXPECT(continuousParentKeys.size() == 1);
  EXPECT(continuousParentKeys[0] == X(0));
}

/* ************************************************************************* */
// Create a likelihood factor for a Gaussian mixture, return a Mixture factor.
GaussianMixtureFactor::shared_ptr likelihood(const GaussianMixture& gm,
                                             const VectorValues& frontals) {
  // TODO(dellaert): check that values has all frontals
  const DiscreteKeys discreteParentKeys = gm.discreteKeys();
  const KeyVector continuousParentKeys = continuousParents(gm);
  const GaussianMixtureFactor::Factors likelihoods(
      gm.conditionals(),
      [&](const GaussianConditional::shared_ptr& conditional) {
        return conditional->likelihood(frontals);
      });
  return boost::make_shared<GaussianMixtureFactor>(
      continuousParentKeys, discreteParentKeys, likelihoods);
}

/// Check that likelihood returns a mixture factor on the parents.
TEST(GaussianMixture, Likelihood) {
  const GaussianMixture gm = createSimpleGaussianMixture();

  // Call the likelihood function:
  VectorValues measurements;
  measurements.insert(Z(0), Vector1(0));
  const auto factor = likelihood(gm, measurements);

  // Check that the factor is a mixture factor on the parents.
  // Loop over all discrete assignments over the discrete parents:
  const DiscreteKeys discreteParentKeys = gm.discreteKeys();

  // Apply the likelihood function to all conditionals:
  const GaussianMixtureFactor::Factors factors(
      gm.conditionals(),
      [measurements](const GaussianConditional::shared_ptr& conditional) {
        return conditional->likelihood(measurements);
      });
  const GaussianMixtureFactor expected({X(0)}, {mode}, factors);
  EXPECT(assert_equal(*factor, expected));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
