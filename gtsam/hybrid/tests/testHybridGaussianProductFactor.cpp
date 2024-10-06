/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridGaussianProductFactor.cpp
 * @brief   Unit tests for HybridGaussianProductFactor
 * @author  Frank Dellaert
 * @date    October 2024
 */

#include "gtsam/inference/Key.h"
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianProductFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/JacobianFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

#include <memory>

using namespace std;
using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace examples {
static const DiscreteKey m1(M(1), 2), m2(M(2), 3);

auto A1 = Matrix::Zero(2, 1);
auto A2 = Matrix::Zero(2, 2);
auto b = Matrix::Zero(2, 1);

auto f10 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
auto f11 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);

auto A3 = Matrix::Zero(2, 3);
auto f20 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
auto f21 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
auto f22 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);

HybridGaussianFactor hybridFactorA(m1, {f10, f11});
HybridGaussianFactor hybridFactorB(m2, {f20, f21, f22});
// Simulate a pruned hybrid factor, in this case m2==1 is nulled out.
HybridGaussianFactor prunedFactorB(m2, {f20, nullptr, f22});
} // namespace examples

/* ************************************************************************* */
// Constructor
TEST(HybridGaussianProductFactor, Construct) {
  HybridGaussianProductFactor product;
}

/* ************************************************************************* */
// Add two Gaussian factors and check only one leaf in tree
TEST(HybridGaussianProductFactor, AddTwoGaussianFactors) {
  using namespace examples;

  HybridGaussianProductFactor product;
  product += f10;
  product += f11;

  // Check that the product has only one leaf and no discrete variables.
  EXPECT_LONGS_EQUAL(1, product.nrLeaves());
  EXPECT(product.labels().empty());

  // Retrieve the single leaf
  auto leaf = product(Assignment<Key>());

  // Check that the leaf contains both factors
  EXPECT_LONGS_EQUAL(2, leaf.size());
  EXPECT(leaf.at(0) == f10);
  EXPECT(leaf.at(1) == f11);
}

/* ************************************************************************* */
// Add two GaussianConditionals and check the resulting tree
TEST(HybridGaussianProductFactor, AddTwoGaussianConditionals) {
  // Create two GaussianConditionals
  Vector1 d(1.0);
  Matrix11 R = I_1x1, S = I_1x1;
  auto gc1 = std::make_shared<GaussianConditional>(X(1), d, R, X(2), S);
  auto gc2 = std::make_shared<GaussianConditional>(X(2), d, R);

  // Create a HybridGaussianProductFactor and add the conditionals
  HybridGaussianProductFactor product;
  product += std::static_pointer_cast<GaussianFactor>(gc1);
  product += std::static_pointer_cast<GaussianFactor>(gc2);

  // Check that the product has only one leaf and no discrete variables
  EXPECT_LONGS_EQUAL(1, product.nrLeaves());
  EXPECT(product.labels().empty());

  // Retrieve the single leaf
  auto leaf = product(Assignment<Key>());

  // Check that the leaf contains both conditionals
  EXPECT_LONGS_EQUAL(2, leaf.size());
  EXPECT(leaf.at(0) == gc1);
  EXPECT(leaf.at(1) == gc2);
}

/* ************************************************************************* */
// Check AsProductFactor
TEST(HybridGaussianProductFactor, AsProductFactor) {
  using namespace examples;
  auto product = hybridFactorA.asProductFactor();

  // Let's check that this worked:
  Assignment<Key> mode;
  mode[m1.first] = 1;
  auto actual = product(mode);
  EXPECT(actual.at(0) == f11);
}

/* ************************************************************************* */
// "Add" one hybrid factors together.
TEST(HybridGaussianProductFactor, AddOne) {
  using namespace examples;
  HybridGaussianProductFactor product;
  product += hybridFactorA;

  // Let's check that this worked:
  Assignment<Key> mode;
  mode[m1.first] = 1;
  auto actual = product(mode);
  EXPECT(actual.at(0) == f11);
}

/* ************************************************************************* */
// "Add" two HFG together.
TEST(HybridGaussianProductFactor, AddTwo) {
  using namespace examples;

  // Create product of two hybrid factors: it will be a decision tree now on
  // both discrete variables m1 and m2:
  HybridGaussianProductFactor product;
  product += hybridFactorA;
  product += hybridFactorB;

  // Let's check that this worked:
  auto actual00 = product({{M(1), 0}, {M(2), 0}});
  EXPECT(actual00.at(0) == f10);
  EXPECT(actual00.at(1) == f20);

  auto actual12 = product({{M(1), 1}, {M(2), 2}});
  EXPECT(actual12.at(0) == f11);
  EXPECT(actual12.at(1) == f22);
}

/* ************************************************************************* */
// "Add" two HFG together.
TEST(HybridGaussianProductFactor, AddPruned) {
  using namespace examples;

  // Create product of two hybrid factors: it will be a decision tree now on
  // both discrete variables m1 and m2:
  HybridGaussianProductFactor product;
  product += hybridFactorA;
  product += prunedFactorB;
  EXPECT_LONGS_EQUAL(6, product.nrLeaves());

  auto pruned = product.removeEmpty();
  EXPECT_LONGS_EQUAL(5, pruned.nrLeaves());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
