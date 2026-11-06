/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/GaussianISAM2.h>

#include <atomic>
#include <stdexcept>

using namespace gtsam;

/* ************************************************************************* */
namespace branching {

const Ordering ordering{0, 1, 2, 3};
const Matrix11 unit{{1.0}};

GaussianFactorGraph graph() {
  GaussianFactorGraph result;
  for (Key leaf = 0; leaf < 3; ++leaf)
    result.emplace_shared<JacobianFactor>(leaf, unit, 3, -unit,
                                         Vector1{double(leaf + 1)});
  result.emplace_shared<JacobianFactor>(3, unit, Vector1{0.0});
  return result;
}

// Additions, replacements, and repeated removals match fresh batch elimination.
TEST(GaussianISAM2Incremental, BatchAgreement) {
  GaussianFactorGraph factors = graph();
  GaussianISAM2 solver(ordering, EliminateQR);
  EXPECT(solver.update(factors) == FactorIndices({0, 1, 2, 3}));
  EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));

  for (size_t repeat = 0; repeat < 3; ++repeat) {
    GaussianFactorGraph additions;
    additions.emplace_shared<JacobianFactor>(0, unit, Vector1{4.0});
    FactorIndices indices = solver.update(additions);
    EXPECT(indices == FactorIndices({factors.size()}));
    factors.push_back(additions);
    EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));

    additions[0] = std::make_shared<JacobianFactor>(0, unit, Vector1{-2.0});
    const FactorIndices replacement = solver.update(additions, indices);
    EXPECT(replacement == FactorIndices({factors.size()}));
    factors.remove(indices[0]);
    factors.push_back(additions);
    EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));

    EXPECT(solver.update({}, replacement).empty());
    factors.remove(replacement[0]);
    EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));
  }
}

// Updating one branch reuses another branch's clique and cached separator.
TEST(GaussianISAM2Incremental, ReusesUntouchedSubtree) {
  std::atomic<size_t> eliminations{0};
  GaussianISAM2 solver(ordering, [&](const GaussianFactorGraph& factors,
                                    const Ordering& frontalKeys) {
    ++eliminations;
    return EliminateQR(factors, frontalKeys);
  });
  GaussianFactorGraph factors = graph();
  solver.update(factors);
  const auto untouched = solver[1], affected = solver[0], root = solver[3];
  const size_t initialEliminations = eliminations.load();
  EXPECT(untouched != root);
  EXPECT(solver.update({}).empty());
  EXPECT_LONGS_EQUAL(initialEliminations, eliminations.load());
  EXPECT(solver[0] == affected && solver[1] == untouched && solver[3] == root);

  GaussianFactorGraph additions;
  additions.emplace_shared<JacobianFactor>(0, unit, Vector1{5.0},
                                           noiseModel::Constrained::All(1));
  const FactorIndices added = solver.update(additions);
  factors.push_back(additions);
  EXPECT(solver[1] == untouched);
  EXPECT(solver[0] != affected && solver[3] != root);
  EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));

  solver.update({}, added);
  factors.remove(added[0]);
  EXPECT(solver[1] == untouched);
  EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));
}

// Hessian-native objective factors support mixed hard-row updates as well.
TEST(GaussianISAM2Incremental, HessianNative) {
  GaussianFactorGraph factors;
  for (const auto& factor : graph())
    factors.emplace_shared<HessianFactor>(*factor);
  GaussianISAM2 solver(ordering);
  solver.update(factors);
  EXPECT(assert_equal(factors.optimize(ordering), solver.optimize()));
  GaussianFactorGraph additions;
  additions.emplace_shared<JacobianFactor>(0, unit, Vector1{2.0},
                                           noiseModel::Constrained::All(1));
  const FactorIndices added = solver.update(additions);
  factors.push_back(additions);
  EXPECT(assert_equal(factors.optimize(ordering), solver.optimize()));
  solver.update({}, added);
  factors.remove(added[0]);
  EXPECT(assert_equal(factors.optimize(ordering), solver.optimize()));
}

}  // namespace branching
/* ************************************************************************* */
namespace constrained {

// Weighted tiny coefficients survive adding and removing opposing exact rows.
TEST(GaussianISAM2Incremental, WeightedAndDependentRows) {
  const Ordering ordering{0, 1};
  const Matrix11 unit{{1.0}}, tiny{{1e-10}};
  const auto hard = noiseModel::Constrained::All(1);
  GaussianFactorGraph factors;
  factors.emplace_shared<JacobianFactor>(0, unit, Vector1{0.0}, hard);
  factors.emplace_shared<JacobianFactor>(0, tiny, 1, tiny, Vector1{2e-10},
                                         noiseModel::Isotropic::Sigma(1, 1e-10));
  GaussianISAM2 solver(ordering, EliminateQR);
  solver.update(factors);
  EXPECT(assert_equal(Vector1{2.0}, solver.optimize().at(1)));

  GaussianFactorGraph additions;
  additions.emplace_shared<JacobianFactor>(1, unit, Vector1{1.0}, hard);
  additions.emplace_shared<JacobianFactor>(1, -unit, Vector1{-1.0}, hard);
  const FactorIndices added = solver.update(additions);
  factors.push_back(additions);
  EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));
  EXPECT(assert_equal(Vector1{1.0}, solver.optimize().at(1)));
  solver.update({}, added);
  for (size_t index : added) factors.remove(index);
  EXPECT(assert_equal(factors.optimize(ordering, EliminateQR), solver.optimize()));
  EXPECT(assert_equal(Vector1{2.0}, solver.optimize().at(1)));
}

}  // namespace constrained
/* ************************************************************************* */
namespace validation {

// Invalid edits leave a usable tree; replacement preserves the fixed key set.
TEST(GaussianISAM2Incremental, InvalidUpdatesAreAtomic) {
  const Ordering ordering{0, 1};
  const Matrix11 unit{{1.0}};
  GaussianFactorGraph factors;
  factors.emplace_shared<JacobianFactor>(0, unit, Vector1{1.0});
  factors.emplace_shared<JacobianFactor>(1, unit, Vector1{2.0});
  GaussianISAM2 solver(ordering);
  solver.update(factors);
  const auto first = solver[0], second = solver[1];
  const VectorValues expected = solver.optimize();
  CHECK_EXCEPTION(solver.update({}, {9}), std::invalid_argument);
  CHECK_EXCEPTION(solver.update({}, {0, 0}), std::invalid_argument);
  CHECK_EXCEPTION(solver.update({}, {0}), std::invalid_argument);
  GaussianFactorGraph additions;
  additions.emplace_shared<JacobianFactor>(2, unit, Vector1{0.0});
  CHECK_EXCEPTION(solver.update(additions), std::invalid_argument);
  additions[0] = std::make_shared<JacobianFactor>(0, Matrix22::Identity(),
                                                Vector2::Zero());
  CHECK_EXCEPTION(solver.update(additions, {0}), std::invalid_argument);
  EXPECT(solver[0] == first && solver[1] == second);
  EXPECT(assert_equal(expected, solver.optimize()));

  additions[0] = std::make_shared<JacobianFactor>(0, unit, Vector1{3.0});
  EXPECT(solver.update(additions, {0}) == FactorIndices({2}));
  EXPECT(assert_equal(Vector1{3.0}, solver.optimize().at(0)));
  EXPECT(solver[1] == second);
  CHECK_EXCEPTION(solver.update({}, {0}), std::invalid_argument);
}

// Empty graphs are valid, while an initial nonempty graph must cover ordering.
TEST(GaussianISAM2Incremental, Initialization) {
  GaussianISAM2 empty(Ordering{});
  EXPECT(empty.update({}).empty());
  EXPECT_LONGS_EQUAL(0, empty.optimize().size());
  CHECK_EXCEPTION(GaussianISAM2(Ordering({0, 0})), std::invalid_argument);
  GaussianISAM2 solver(Ordering{0, 1});
  CHECK_EXCEPTION(solver.optimize(), std::logic_error);
  GaussianFactorGraph incomplete;
  incomplete.emplace_shared<JacobianFactor>(0, Matrix11{{1.0}}, Vector1{0.0});
  CHECK_EXCEPTION(solver.update(incomplete), std::invalid_argument);
  incomplete.emplace_shared<JacobianFactor>(1, Matrix11{{1.0}}, Vector1{1.0});
  solver.update(incomplete);
  EXPECT(assert_equal(incomplete.optimize(), solver.optimize()));
}

}  // namespace validation
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
