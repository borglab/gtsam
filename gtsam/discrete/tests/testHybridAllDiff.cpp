/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridAllDiff.cpp
 * @brief   Test AllDiff in a hybrid Gaussian factor graph.
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/AllDiff.h>
#include <gtsam/discrete/SingleValue.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using symbol_shorthand::A;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace hybrid_all_diff {

class AllDiffWithoutDecisionTreeConversion : public AllDiff {
 public:
  using AllDiff::AllDiff;

  DecisionTreeFactor toDecisionTreeFactor() const override {
    throw std::runtime_error(
        "hybrid elimination requested AllDiff::toDecisionTreeFactor()");
  }
};

HybridGaussianFactorGraph MakeDataAssociationGraph() {
  const std::vector<Vector2> predicted{{0.0, 0.0}, {3.0, 0.0}, {0.0, 3.0}};
  const std::vector<Vector2> measurements{{2.8, 0.2}, {-0.1, 3.1}, {0.2, -0.1}};
  const auto priorModel = noiseModel::Isotropic::Sigma(2, 1.5);
  const auto measurementModel = noiseModel::Isotropic::Sigma(2, 1.5);

  HybridGaussianFactorGraph graph;
  for (size_t i = 0; i < 3; ++i) {
    graph.emplace_shared<JacobianFactor>(X(i), I_2x2, predicted[i], priorModel);

    std::vector<GaussianFactor::shared_ptr> components;
    for (const Vector2& measurement : measurements) {
      components.emplace_back(std::make_shared<JacobianFactor>(
          X(i), I_2x2, measurement, measurementModel));
    }
    graph.emplace_shared<HybridGaussianFactor>(DiscreteKey(A(i), 3),
                                               components);
  }
  return graph;
}

// Verifies that the sparse table conversion agrees with the existing decision
// tree conversion for uniform and unequal cardinalities.
TEST(HybridAllDiff, SparseTableConversion) {
  for (const DiscreteKeys& keys :
       {DiscreteKeys{{A(0), 3}, {A(1), 3}, {A(2), 3}},
        DiscreteKeys{{A(0), 2}, {A(1), 3}, {A(2), 4}}}) {
    const AllDiff allDiff(keys);
    const TableFactor expected(allDiff.toDecisionTreeFactor());
    const TableFactor actual = allDiff.toTableFactor();
    EXPECT(assert_equal(expected, actual));
  }
}

// Verifies both multiplication orders preserve TableFactor and avoid the
// decision-tree conversion even when the table is missing AllDiff variables.
TEST(HybridAllDiff, TableMultiplicationAvoidsDecisionTree) {
  const DiscreteKeys keys{{A(0), 3}, {A(1), 3}, {A(2), 3}};
  const auto constraint =
      std::make_shared<AllDiffWithoutDecisionTreeConversion>(keys);
  const auto unary = std::make_shared<TableFactor>(
      DiscreteKey(A(0), 3), std::vector<double>{1.0, 2.0, 3.0});
  const TableFactor expected = *unary * constraint->toTableFactor();

  const auto constraintFirst =
      std::dynamic_pointer_cast<TableFactor>(constraint->multiply(unary));
  const auto tableFirst =
      std::dynamic_pointer_cast<TableFactor>(unary->multiply(constraint));

  CHECK(constraintFirst);
  CHECK(tableFirst);
  EXPECT(assert_equal(expected, *constraintFirst));
  EXPECT(assert_equal(expected, *tableFirst));
  LONGS_EQUAL(6, tableFirst->nrValues());
}

// Verifies the generic constraint fallback retains TableFactor in both
// multiplication orders.
TEST(HybridAllDiff, GenericConstraintTableMultiplication) {
  const DiscreteKey key(A(0), 3);
  const auto constraint = std::make_shared<SingleValue>(key, 1);
  const auto table =
      std::make_shared<TableFactor>(key, std::vector<double>{1.0, 2.0, 3.0});
  const TableFactor expected(key, std::vector<double>{0.0, 2.0, 0.0});

  const auto constraintFirst =
      std::dynamic_pointer_cast<TableFactor>(constraint->multiply(table));
  const auto tableFirst =
      std::dynamic_pointer_cast<TableFactor>(table->multiply(constraint));

  CHECK(constraintFirst);
  CHECK(tableFirst);
  EXPECT(assert_equal(expected, *constraintFirst));
  EXPECT(assert_equal(expected, *tableFirst));
}

// Verifies sequential inference recovers the expected one-to-one association
// and continuous estimates while preserving the sparse table representation.
TEST(HybridAllDiff, SequentialDataAssociation) {
  HybridGaussianFactorGraph graph = MakeDataAssociationGraph();
  graph.emplace_shared<AllDiff>(DiscreteKeys{{A(0), 3}, {A(1), 3}, {A(2), 3}});

  const Ordering ordering{X(0), X(1), X(2), A(0), A(1), A(2)};
  const HybridValues result = graph.eliminateSequential(ordering)->optimize();

  LONGS_EQUAL(2, result.discrete().at(A(0)));
  LONGS_EQUAL(0, result.discrete().at(A(1)));
  LONGS_EQUAL(1, result.discrete().at(A(2)));
  EXPECT(assert_equal(Vector2{0.1, -0.05}, result.continuous().at(X(0))));
  EXPECT(assert_equal(Vector2{2.9, 0.1}, result.continuous().at(X(1))));
  EXPECT(assert_equal(Vector2{-0.05, 3.05}, result.continuous().at(X(2))));
}

// Verifies hybrid elimination no longer requests a decision-tree conversion.
TEST(HybridAllDiff, NativeEliminationAvoidsDecisionTree) {
  HybridGaussianFactorGraph graph = MakeDataAssociationGraph();
  graph.emplace_shared<AllDiffWithoutDecisionTreeConversion>(
      DiscreteKeys{{A(0), 3}, {A(1), 3}, {A(2), 3}});

  const Ordering ordering{X(0), X(1), X(2), A(0), A(1), A(2)};
  const HybridValues sequential =
      graph.eliminateSequential(ordering)->optimize();
  const HybridValues multifrontal =
      graph.eliminateMultifrontal(ordering)->optimize();

  for (const HybridValues* result : {&sequential, &multifrontal}) {
    LONGS_EQUAL(2, result->discrete().at(A(0)));
    LONGS_EQUAL(0, result->discrete().at(A(1)));
    LONGS_EQUAL(1, result->discrete().at(A(2)));
  }
}

}  // namespace hybrid_all_diff
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
