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

#include <limits>

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

// A cached residual correction preserves accuracy along nearly flat affine faces.
TEST(SparseQpSolver, WeakAffineFaceResidualAccuracy) {
  const Vector direction{{1, -2, 4}}, target{{10, -30, 20}};
  const Vector normal = Vector::Constant(3, .008);
  const Matrix coupling = direction * normal.transpose();
  const double weight = 1e-9;
  for (bool squareRoot : {false, true}) {
    for (double offset : {0.0, .5}) {
      for (bool incremental : {false, true}) {
        SparseQpData data;
        data.keyDimensions = {{0, 3}, {1, 3}};
        data.ordering = Ordering{0, 1};
        data.regularization = weight;
        data.useIncremental = incremental;
        if (squareRoot)
          data.objective.emplace_shared<JacobianFactor>(
              0, Matrix::Identity(3, 3), target);
        else
          data.objective.emplace_shared<HessianFactor>(
              0, Matrix::Identity(3, 3), target, 0.0);
        const Vector rhs = offset * direction;
        data.equalities.emplace_shared<JacobianFactor>(
            0, Matrix::Identity(3, 3), 1, -coupling, rhs,
            noiseModel::Constrained::All(3));
        VectorValues center;
        center.insert(0, Vector::Zero(3));
        center.insert(1, Vector::Zero(3));
        const auto result = SparseQpSolver(data).solve({}, center);
        // Eliminating state analytically leaves one observable force direction;
        // the supplied proximal weight selects zero in its two null directions.
        const double scale = direction.dot(target - (1 + weight) * rhs) /
            (weight + (1 + weight) * direction.squaredNorm() * normal.squaredNorm());
        const Vector error = result.at(1) - normal * scale;
        // The two null directions have curvature weight, so absolute solution
        // error is amplified by the reduced system's condition number. Check
        // forward error at that scale, and stationarity independently below.
        const double condition = 1 + (1 + weight) *
            direction.squaredNorm() * normal.squaredNorm() / weight;
        const double tolerance = std::numeric_limits<double>::epsilon() *
            condition * (normal * scale).lpNorm<Eigen::Infinity>();
        CHECK(error.lpNorm<Eigen::Infinity>() < tolerance);
        const Vector gradient = weight * result.at(1) +
            coupling.transpose() * ((1 + weight) * result.at(0) - target);
        CHECK(gradient.lpNorm<Eigen::Infinity>() < 1e-12);
        EXPECT(assert_equal(rhs, Vector(result.at(0) - coupling * result.at(1)), 1e-12));
      }
    }
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
