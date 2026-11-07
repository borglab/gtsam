/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testSparseQpSolver.cpp
 * Check the extracted sparse workspace independently of active-set decisions.
 * Batch comparisons exercise reference-center consistency, original row
 * identities, square-root costs, and rebuilding when graph variables change.
 */

#include <gtsam/base/Testable.h>
#include <gtsam/constrained/internal/SparseQpSolver.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using internal::SparseQpData;
using internal::SparseQpSolver;

/* ************************************************************************* */
namespace sparse_working_system {

SparseQpData problem(double regularization) {
  SparseQpData data;
  data.keyDimensions = {{0, 1}, {1, 1}};
  data.ordering = Ordering{1, 0};
  data.regularization = regularization;
  data.objective.emplace_shared<JacobianFactor>(
      0, Matrix{{1.0}}, Vector{{0.0}});
  data.objective.emplace_shared<JacobianFactor>(
      1, Matrix{{1e-10}}, Vector{{2e-10}});
  data.equalities.emplace_shared<JacobianFactor>(
      0, Matrix{{1.0}}, Vector{{1.0}}, noiseModel::Constrained::All(1));
  for (double scale : {1.0, 2.0, -1.0})
    data.inequalities.emplace_shared<JacobianFactor>(
        1, Matrix{{scale}}, Vector{{scale * .4}},
        noiseModel::Constrained::All(1));
  return data;
}

// Changed centers and repeated dependent/opposing toggles match batch solves.
TEST(SparseQpSolver, CentersAndDependentToggles) {
  const std::vector<std::vector<bool>> masks{
      {false, false, false}, {true, false, false}, {true, true, true},
      {false, true, true}, {false, true, false}, {false, false, false},
      {true, true, false}, {false, true, false}};
  for (double weight : {0.0, .2}) {
    auto data = problem(weight);
    SparseQpSolver incremental(data);
    data.useIncremental = false;
    SparseQpSolver batch(data);
    const auto objective =
        std::dynamic_pointer_cast<JacobianFactor>(data.objective.at(1));
    const Vector originalRhs = objective->getb();
    for (size_t i = 0; i < masks.size(); ++i) {
      for (double scale : {1.0, .5}) {
        VectorValues center;
        center.insert(0, Vector{{scale * double(i) / 3}});
        center.insert(1, Vector{{-scale * double(i)}});
        const auto actual = incremental.solve(masks[i], center);
        EXPECT(assert_equal(batch.solve(masks[i], center), actual, 1e-9));
        EXPECT_DOUBLES_EQUAL(1, actual.at(0)(0), 1e-10);
      }
    }
    EXPECT(assert_equal(originalRhs, Vector(objective->getb()), 0));
  }
}

// Constraint-only variables entering and leaving force a fresh restricted tree.
TEST(SparseQpSolver, VariableSetChanges) {
  SparseQpData data;
  data.keyDimensions = {{0, 1}, {1, 1}};
  data.ordering = Ordering{1, 0};
  data.objective.emplace_shared<JacobianFactor>(
      0, Matrix{{1.0}}, Vector{{2.0}});
  data.inequalities.emplace_shared<JacobianFactor>(
      1, Matrix{{1.0}}, Vector{{3.0}}, noiseModel::Constrained::All(1));
  SparseQpSolver incremental(data);
  data.useIncremental = false;
  SparseQpSolver batch(data);
  VectorValues center;
  center.insert(0, Vector{{0.0}});
  center.insert(1, Vector{{0.0}});
  for (bool active : {false, true, false, true, true, false}) {
    const auto actual = incremental.solve({active}, center);
    EXPECT(assert_equal(batch.solve({active}, center), actual, 1e-10));
    EXPECT(actual.exists(1) == active);
    EXPECT_DOUBLES_EQUAL(2, actual.at(0)(0), 1e-10);
    if (active) EXPECT_DOUBLES_EQUAL(3, actual.at(1)(0), 1e-10);
  }
}

// Hessian-native objectives preserve the existing preferred-elimination path.
TEST(SparseQpSolver, HessianNativeCosts) {
  for (double weight : {0.0, .2}) {
    SparseQpData data;
    data.keyDimensions = {{0, 1}};
    data.regularization = weight;
    data.objective.emplace_shared<HessianFactor>(
        0, Matrix{{2.0}}, Vector{{4.0}}, 0.0);
    data.inequalities.emplace_shared<JacobianFactor>(
        0, Matrix{{1.0}}, Vector{{1.0}}, noiseModel::Constrained::All(1));
    SparseQpSolver incremental(data);
    data.useIncremental = false;
    SparseQpSolver batch(data);
    for (bool active : {false, true, false, true}) {
      VectorValues center;
      center.insert(0, Vector{{active ? .3 : .7}});
      EXPECT(assert_equal(batch.solve({active}, center),
                          incremental.solve({active}, center), 1e-10));
    }
  }
}

}  // namespace sparse_working_system
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
