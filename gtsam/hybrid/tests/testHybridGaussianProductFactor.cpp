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

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianProductFactor.h>
#include <gtsam/inference/Key.h>
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

const auto A1 = Matrix::Zero(2, 1);
const auto A2 = Matrix::Zero(2, 2);
const auto b = Matrix::Zero(2, 1);

const auto f10 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
const auto f11 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
const HybridGaussianFactor hybridFactorA(m1, {{f10, 10}, {f11, 11}});

const auto A3 = Matrix::Zero(2, 3);
const auto f20 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
const auto f21 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
const auto f22 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);

const HybridGaussianFactor hybridFactorB(m2, {{f20, 20}, {f21, 21}, {f22, 22}});
// Simulate a pruned hybrid factor, in this case m2==1 is nulled out.
const HybridGaussianFactor prunedFactorB(m2, {{f20, 20}, {nullptr, 1000}, {f22, 22}});
}  // namespace examples

/* ************************************************************************* */
// Constructor
TEST(HybridGaussianProductFactor, Construct) { HybridGaussianProductFactor product; }

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
  EXPECT_LONGS_EQUAL(2, leaf.first.size());
  EXPECT(leaf.first.at(0) == f10);
  EXPECT(leaf.first.at(1) == f11);
  EXPECT_DOUBLES_EQUAL(0, leaf.second, 1e-9);
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
  EXPECT_LONGS_EQUAL(2, leaf.first.size());
  EXPECT(leaf.first.at(0) == gc1);
  EXPECT(leaf.first.at(1) == gc2);
  EXPECT_DOUBLES_EQUAL(0, leaf.second, 1e-9);
}

/* ************************************************************************* */
// Check AsProductFactor
TEST(HybridGaussianProductFactor, AsProductFactor) {
  using namespace examples;
  auto product = hybridFactorA.asProductFactor();

  // Let's check that this worked:
  Assignment<Key> mode;
  mode[m1.first] = 0;
  auto actual = product(mode);
  EXPECT(actual.first.at(0) == f10);
  EXPECT_DOUBLES_EQUAL(10, actual.second, 1e-9);

  // TODO(Frank): when killed hiding, f11 should also be there
}

/* ************************************************************************* */
// "Add" one hybrid factors together.
TEST(HybridGaussianProductFactor, AddOne) {
  using namespace examples;
  HybridGaussianProductFactor product;
  product += hybridFactorA;

  // Let's check that this worked:
  Assignment<Key> mode;
  mode[m1.first] = 0;
  auto actual = product(mode);
  EXPECT(actual.first.at(0) == f10);
  EXPECT_DOUBLES_EQUAL(10, actual.second, 1e-9);

  // TODO(Frank): when killed hiding, f11 should also be there
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
  EXPECT(actual00.first.at(0) == f10);
  EXPECT(actual00.first.at(1) == f20);
  EXPECT_DOUBLES_EQUAL(10 + 20, actual00.second, 1e-9);

  auto actual12 = product({{M(1), 1}, {M(2), 2}});
  // TODO(Frank): when killed hiding, these should also equal:
  // EXPECT(actual12.first.at(0) == f11);
  // EXPECT(actual12.first.at(1) == f22);
  EXPECT_DOUBLES_EQUAL(11 + 22, actual12.second, 1e-9);
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
