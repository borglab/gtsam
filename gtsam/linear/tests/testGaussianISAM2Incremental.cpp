/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGaussianISAM2Incremental.cpp
 * Regression coverage for fixed-order updates, subtree reuse, exact constraints,
 * and cached objective-forcing queries without numerical elimination.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/GaussianISAM2.h>

#include <atomic>
#include <limits>
#include <stdexcept>

using namespace gtsam;

/* ************************************************************************* */
namespace branching {

const Ordering ordering{0, 1, 2, 3};
const Matrix11 unit{{1.0}};

GaussianFactorGraph graph() {
  // Couple three leaves to one anchored root so one branch can remain untouched.
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
    // Add a factor, replace it using a new index, then remove the replacement.
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
  // Record clique identity as well as numerical results before changing factors.
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
  // Preserve Hessian-native costs while toggling an exact Jacobian constraint.
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

  // Invalid removals and incompatible factor replacements must preserve state.
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
  // An empty ordering needs no factorization; nonempty orderings need all keys.
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

/* ************************************************************************* */
namespace objective_forcing {

const Ordering ordering{0, 1, 2, 3};
const Matrix11 unit{{1.0}};

// Change only finite objective RHSs; the graph's coupling keeps three branches.
GaussianFactorGraph graph(const VectorValues& forcing, bool hessian) {
  // Express information-vector changes as weighted finite-factor RHS changes.
  GaussianFactorGraph result = branching::graph();
  for (Key key : ordering) {
    const double sigma = key == 1 ? 0.25 : 2.0;
    const double rhs = forcing.exists(key) ? sigma * sigma * forcing.at(key)(0) : 0.0;
    auto factor = std::make_shared<JacobianFactor>(
        key, unit, Vector1{rhs}, noiseModel::Isotropic::Sigma(1, sigma));
    if (hessian) result.emplace_shared<HessianFactor>(*factor);
    else result.push_back(factor);
  }
  return result;
}

// Arbitrary RHS queries do no elimination or mutation, before/after row toggles.
TEST(GaussianISAM2Incremental, ForcingAndWorkingSetReferenceConsistency) {
  // Compare cached solves to fresh graphs for both objective representations.
  for (bool hessian : {false, true}) {
    std::atomic<size_t> eliminations{0};
    auto eliminate = [hessian, &eliminations](const GaussianFactorGraph& factors,
                                             const Ordering& frontals) {
      ++eliminations;
      return hessian ? EliminatePreferCholesky(factors, frontals)
                     : EliminateQR(factors, frontals);
    };
    GaussianISAM2 solver(ordering, eliminate);
    const GaussianFactorGraph reference = graph({}, hessian);
    solver.update(reference);
    GaussianFactorGraph hardRows;
    const auto hard = noiseModel::Constrained::All(1);
    hardRows.emplace_shared<JacobianFactor>(0, unit, 3, unit, Vector1{0.75}, hard);
    hardRows.emplace_shared<JacobianFactor>(0, -unit, 3, -unit, Vector1{-0.75}, hard);
    FactorIndices active;
    for (size_t toggle = 0; toggle < 6; ++toggle) {
      if (toggle % 2 == 0) active = solver.update(hardRows);
      else solver.update({}, active);
      const auto untouched = solver[1];
      const auto separator = untouched->cachedFactor();
      const size_t count = eliminations.load();
      for (double offset : {2.0, -3.0, 0.0, 0.01}) {
        const VectorValues forcing{{0, Vector1{offset}}, {3, Vector1{-0.7 * offset}}};
        GaussianFactorGraph expected = graph(forcing, hessian);
        if (toggle % 2 == 0) expected.push_back(hardRows);
        EXPECT(assert_equal(expected.optimize(ordering, eliminate),
                            solver.optimize(forcing), 1e-9));
        // The batch comparison uses its own elimination; the query uses none.
        const size_t beforeQuery = eliminations.load();
        solver.optimize(forcing);
        EXPECT_LONGS_EQUAL(beforeQuery, eliminations.load());
        EXPECT(solver[1] == untouched && untouched->cachedFactor() == separator);
      }
      EXPECT(eliminations.load() > count);  // Only the independent batch calls.
      GaussianFactorGraph original = reference;
      if (toggle % 2 == 0) original.push_back(hardRows);
      EXPECT(assert_equal(original.optimize(ordering, eliminate), solver.optimize(), 1e-9));
    }
  }
}

// Non-unit weighted rows and a weak direction obey an affine exact equation.
TEST(GaussianISAM2Incremental, ForcingWeightedWeakAndAffineRows) {
  // Combine weighted weak observations with an unchanged exact affine equation.
  const Matrix22 coefficients{{1e-10, 0.0}, {0.0, 1e-15}};
  const Vector2 sigmas{1e-10, 1e-10};
  const auto noise = noiseModel::Diagonal::Sigmas(sigmas);
  const Matrix12 normal{{1.0, 1.0}};
  GaussianFactorGraph reference;
  reference.emplace_shared<JacobianFactor>(0, coefficients, Vector2{2e-10, -3e-15}, noise);
  reference.emplace_shared<JacobianFactor>(0, normal, Vector1{0.5},
                                          noiseModel::Constrained::All(1));
  size_t eliminations = 0;
  GaussianISAM2 solver(Ordering{0}, [&](const GaussianFactorGraph& factors,
                                      const Ordering& frontals) {
    ++eliminations;
    return EliminateQR(factors, frontals);
  });
  solver.update(reference);
  for (double shift : {-0.3, 0.1, 0.0}) {
    // A^T W^2 delta_b is the forcing in the original variable coordinates.
    const Vector2 delta{shift * 1e-10, shift * 1e-15};
    const Vector2 forcing = coefficients.transpose() *
                           delta.cwiseQuotient(sigmas.cwiseProduct(sigmas));
    GaussianFactorGraph changed;
    changed.emplace_shared<JacobianFactor>(0, coefficients,
        Vector2(Vector2{2e-10, -3e-15} + delta), noise);
    changed.push_back(reference[1]);
    EXPECT(assert_equal(changed.optimize(Ordering{0}, EliminateQR),
                        solver.optimize(VectorValues{{0, forcing}}), 1e-9));
    EXPECT_LONGS_EQUAL(1, eliminations);
  }
}

// Bad forcing is rejected without changing the reference tree or RHS.
TEST(GaussianISAM2Incremental, InvalidForcingAndEmptyGraph) {
  GaussianISAM2 solver(ordering, EliminateQR);
  CHECK_EXCEPTION(solver.optimize(VectorValues{}), std::logic_error);
  solver.update(graph({}, false));
  const auto root = solver[3];
  const VectorValues reference = solver.optimize();

  // Rejected forcing blocks must leave the cached tree and reference RHS intact.
  for (const VectorValues& invalid : std::vector<VectorValues>{
      {{99, Vector1{0.0}}}, {{0, Vector2{0.0, 1.0}}},
      {{0, Vector1{std::numeric_limits<double>::quiet_NaN()}}},
      {{3, Vector1{std::numeric_limits<double>::infinity()}}}}) {
    CHECK_EXCEPTION(solver.optimize(invalid), std::invalid_argument);
    EXPECT(solver[3] == root);
    EXPECT(assert_equal(reference, solver.optimize()));
  }
  EXPECT(assert_equal(reference, solver.optimize(VectorValues{})));
  GaussianISAM2 empty(Ordering{});
  EXPECT_LONGS_EQUAL(0, empty.optimize(VectorValues{}).size());
  CHECK_EXCEPTION(empty.optimize(VectorValues{{0, Vector1{1.0}}}), std::invalid_argument);
}

}  // namespace objective_forcing
/* ************************************************************************* */

int main() {
  // Run all incremental-update and cached-forcing regression cases.
  TestResult result;
  return TestRegistry::runAllTests(result);
}
