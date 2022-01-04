/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDCGaussianMixtureFactor.cpp
 * @brief   Unit tests for DCGaussianMixtureFactor
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/hybrid/DCGaussianMixtureFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ************************************************************************* */
// Check iterators of empty mixture.
TEST(DCGaussianMixtureFactor, Constructor) {
  DCGaussianMixtureFactor factor;
  DCGaussianMixtureFactor::const_iterator const_it = factor.begin();
  CHECK(const_it == factor.end());
  DCGaussianMixtureFactor::iterator it = factor.begin();
  CHECK(it == factor.end());
}

/* ************************************************************************* */
// "Add" two mixture factors together.
TEST(DCGaussianMixtureFactor, Sum) {
  DiscreteKey m1(1, 2), m2(2, 3);

  auto A1 = Matrix::Zero(2, 1);
  auto A2 = Matrix::Zero(2, 2);
  auto A3 = Matrix::Zero(2, 3);
  auto b = Matrix::Zero(2, 1);
  Vector2 sigmas;
  sigmas << 1, 2;
  auto model = noiseModel::Diagonal::Sigmas(sigmas, true);

  auto f10 = boost::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  auto f11 = boost::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  auto f20 = boost::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f21 = boost::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f22 = boost::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  std::vector<GaussianFactor::shared_ptr> factorsA{f10, f11};
  std::vector<GaussianFactor::shared_ptr> factorsB{f20, f21, f22};

  // TODO(Frank): why specify keys at all? And: keys in factor should be *all*
  // keys, deviating from Kevin's scheme. Should we index DT on DiscreteKey?
  // Design review!
  DCGaussianMixtureFactor mixtureFactorA({X(1), X(2)}, {m1}, factorsA);
  DCGaussianMixtureFactor mixtureFactorB({X(1), X(3)}, {m2}, factorsB);

  // Check that number of keys is 2 // TODO(Frank): it should be 3 !
  EXPECT_LONGS_EQUAL(2, mixtureFactorA.keys().size());

  // Check that number of discrete keys is 1 // TODO(Frank): should not exist?
  EXPECT_LONGS_EQUAL(1, mixtureFactorA.discreteKeys().size());

  // Create sum of two mixture factors: it will be a decision tree now on both
  // discrete variables m1 and m2:
  DCGaussianMixtureFactor::Sum sum;
  sum += mixtureFactorA;
  sum += mixtureFactorB;

  // Let's check that this worked:
  Assignment<Key> mode;
  mode[m1.first] = 1;
  mode[m2.first] = 2;
  auto actual = sum(mode);
  EXPECT(actual.at(0) == f11);
  EXPECT(actual.at(1) == f22);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */