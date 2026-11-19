/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testDenseQpSolver.cpp
 * Regression tests for cached dense QP working-system solves.
 *
 * This file owns the uncached rank-aware QR reference used to check solutions
 * across RHS changes, repeated constraint toggles, and dependent boundaries.
 * Keeping that reference in the test executable allows independent comparisons
 * without a batch-mode switch in the runtime library. Tests also verify weak
 * objective directions and zero numerical factorizations on RHS-only solves.
 */

#include <gtsam/constrained/internal/DenseQpSolver.h>
#include <gtsam/constrained/internal/DenseQpEqualityProjection.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace gtsam;
using internal::DenseQpData;
using internal::DenseQpSolver;

/* ************************************************************************* */
namespace dense_working_system {

/**
 * Uncached rank-aware QR oracle compiled only into this test executable.
 * Each solve refactors the original working problem independently of the
 * production caches, so comparisons can detect stale RHS data and incorrect
 * working-set updates. The oracle preserves the original square-root objective
 * and checks every selected boundary, including redundant rows.
 */
class BatchReference {
 public:
  // Keep the same original square-root objective and requested proximal weight.
  explicit BatchReference(const DenseQpData& data) : data_(data) {
    // Append exactly the requested proximal curvature in square-root form.
    const Eigen::Index n = data_.linear.size();
    const Eigen::Index extra = data_.regularization > 0 ? n : 0;
    objective_ = Matrix::Zero(data_.jacobian.rows() + extra, n);
    objective_.topRows(data_.jacobian.rows()) = data_.jacobian;
    if (extra) objective_.bottomRows(n).diagonal().setConstant(
        std::sqrt(data_.regularization));
  }

  // Recompute the working solution for comparison across centers and row changes.
  Vector solve(const std::vector<bool>& active, const Vector& center) const {
    const auto rows = selected(active);
    const auto system = constraints(rows);
    const auto space = feasibleSpace(system.first, system.second);
    const Vector value = minimize(space.first, space.second, target(center));
    check(value, rows);
    return value;
  }

 private:
  const DenseQpData& data_;
  Matrix objective_;

  // Preserve original equality/inequality ordering in the test oracle.
  std::vector<size_t> selected(const std::vector<bool>& active) const {
    if (active.size() + data_.equalities != size_t(data_.bounds.size()))
      throw std::invalid_argument("Dense QP: invalid working-set mask.");
    std::vector<size_t> rows;
    for (size_t row = 0; row < data_.equalities; ++row) rows.push_back(row);
    for (size_t row = 0; row < active.size(); ++row)
      if (active[row]) rows.push_back(data_.equalities + row);
    return rows;
  }

  // Rebuild the square-root RHS directly from the supplied proximal center.
  Vector target(const Vector& center) const {
    Vector result(objective_.rows());
    result.head(data_.rhs.size()) = data_.rhs;
    if (data_.regularization > 0)
      result.tail(center.size()) = std::sqrt(data_.regularization) * center;
    return result;
  }

  // Normalize for rank detection; retain the original rows for checking.
  std::pair<Matrix, Vector> constraints(
      const std::vector<size_t>& rows) const {
    Matrix normals(rows.size(), data_.normals.cols());
    Vector bounds(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
      const size_t row = rows[i];
      const double scale = data_.normals.row(row).stableNorm();
      normals.row(i) = data_.normals.row(row) / (scale > 0 ? scale : 1);
      bounds(i) = data_.bounds(row) / (scale > 0 ? scale : 1);
    }
    return {normals, bounds};
  }

  // Find an affine feasible point and an orthogonal null-space basis.
  std::pair<Vector, Matrix> feasibleSpace(const Matrix& normals,
                                         const Vector& bounds) const {
    const Eigen::Index n = data_.linear.size();
    if (normals.rows() == 0)
      return {Vector::Zero(n), Matrix::Identity(n, n)};
    Eigen::ColPivHouseholderQR<Matrix> qr(normals.transpose());
    const Eigen::Index rank = qr.rank();
    const Matrix orthogonal = qr.householderQ() * Matrix::Identity(n, n);
    Vector ordered = qr.colsPermutation().transpose() * bounds;
    const Vector coordinates = qr.matrixR().topLeftCorner(rank, rank)
        .transpose().triangularView<Eigen::Lower>().solve(ordered.head(rank));
    return {orthogonal.leftCols(rank) * coordinates,
            orthogonal.rightCols(n - rank)};
  }

  // Solve linear forcing with triangular factors, without normal equations.
  Vector linearCorrection(const Eigen::ColPivHouseholderQR<Matrix>& qr,
                          const Vector& forcing) const {
    const Eigen::Index rank = qr.rank(), n = qr.cols();
    if (forcing.isZero(0)) return Vector::Zero(n);
    const Vector permuted = qr.colsPermutation().transpose() * forcing;
    Vector forward = qr.matrixR().topLeftCorner(rank, rank).transpose()
        .triangularView<Eigen::Lower>().solve(permuted.head(rank));
    const Vector residual = permuted -
        qr.matrixR().topRows(rank).triangularView<Eigen::Upper>().transpose() * forward;
    const double allowance = 64 * std::numeric_limits<double>::epsilon() *
                             std::max(qr.rows(), qr.cols()) * forcing.norm();
    if (residual.norm() > allowance)
      throw std::runtime_error("Dense QP: unbounded working-face linear cost.");
    Vector correction = Vector::Zero(n);
    correction.head(rank) = qr.matrixR().topLeftCorner(rank, rank)
        .triangularView<Eigen::Upper>().solve(forward);
    return qr.colsPermutation() * correction;
  }

  // Refactor the reduced objective on every comparison solve.
  Vector minimize(const Vector& point, const Matrix& nullspace,
                  const Vector& rhs) const {
    if (nullspace.cols() == 0) return point;
    const Matrix full = objective_ * nullspace;
    const Vector target = rhs - objective_ * point;
    // Exactly constant residual rows do not affect feasible motion. Removing
    // only exact zeros avoids mixing their large offsets into weak directions.
    Matrix reduced(full.rows(), full.cols());
    Vector varying(full.rows());
    Eigen::Index count = 0;
    for (Eigen::Index row = 0; row < full.rows(); ++row) {
      if (full.row(row).isZero(0)) continue;
      reduced.row(count) = full.row(row);
      varying(count++) = target(row);
    }
    reduced.conservativeResize(count, Eigen::NoChange);
    varying.conservativeResize(count);
    if (count == 0) {
      if ((nullspace.transpose() * data_.linear).norm() > 0)
        throw std::runtime_error("Dense QP: unbounded working-face linear cost.");
      return point;
    }
    Eigen::ColPivHouseholderQR<Matrix> qr(reduced);
    Vector solution = qr.solve(varying);
    solution += linearCorrection(qr, nullspace.transpose() * data_.linear);
    return point + nullspace * solution;
  }

  // Check all selected original boundaries, including redundant rows.
  void check(const Vector& value, const std::vector<size_t>& rows) const {
    if (!value.allFinite())
      throw std::runtime_error("Dense QP: non-finite working-face solution.");
    for (size_t row : rows) {
      const double scale = 1 + std::abs(data_.bounds(row)) +
          data_.normals.row(row).cwiseAbs().dot(value.cwiseAbs());
      const double allowance = 256 * std::numeric_limits<double>::epsilon() * scale;
      if (std::abs(data_.normals.row(row).dot(value) - data_.bounds(row)) >
          std::max(1e-10, allowance))
        throw std::runtime_error("Dense QP: inconsistent working constraints.");
    }
  }
};

DenseQpData problem(double regularization) {
  DenseQpData data;
  data.jacobian = Matrix{{2, 0}, {0, 3}};
  data.rhs = Vector{{1, -1}};
  data.linear = Vector::Zero(2);
  data.normals = Matrix{{1, 1}, {1, 0}, {2, 0}, {-1, 0}};
  data.bounds = Vector{{1, .4, .8, -.4}};
  data.equalities = 1;
  data.regularization = regularization;
  return data;
}

// Native-Hessian Cholesky shares working-set updates and RHS caches with QR,
// and agrees with an independently refactored square-root working-face solve.
TEST(DenseQpSolver, NativeHessianCholeskyReuse) {
  for (double weight : {0.0, 0.1}) {
    DenseQpData data;
    data.nativeHessian = Matrix{{4, 1}, {1, 3}};
    data.jacobian = Eigen::LLT<Matrix>(data.nativeHessian).matrixU();
    data.rhs = Vector::Zero(2);
    data.linear = Vector2{8, 7};
    data.normals = Matrix{{1, 0}, {2, 0}, {0, 1}};
    data.bounds = Vector3{1, 2, 3};
    data.regularization = weight;
    DenseQpSolver solver(data), dual(data);
    BatchReference reference(data);
    CHECK(dual.prepareDual());
    const std::vector<std::vector<bool>> masks{
        {false, false, false}, {true, false, false}, {true, true, false},
        {false, true, false}, {false, false, false}};
    for (size_t i = 0; i < masks.size(); ++i) {
      const Vector2 center{double(i), -double(i)};
      EXPECT(assert_equal(reference.solve(masks[i], center),
                          solver.solve(masks[i], center), 1e-9));
      EXPECT(assert_equal(reference.solve(masks[i], center * .5),
                          solver.solve(masks[i], center * .5), 1e-9));
      Matrix hessian = data.nativeHessian;
      hessian.diagonal().array() += weight;
      const Vector2 forcing{2, -1};
      EXPECT(assert_equal(Vector(hessian.ldlt().solve(data.linear + forcing + weight * center)),
                          dual.solveDual(center, forcing), 1e-10));
      EXPECT(assert_equal(Vector(hessian.ldlt().solve(forcing)),
                          dual.dualResponse(forcing), 1e-10));
    }
    CHECK(solver.statistics().choleskyFactorizations == 1);
    CHECK(solver.statistics().objectiveFactorizations == 1);
    CHECK(dual.statistics().choleskyFactorizations == 1);
    CHECK(dual.statistics().numericalFactorizations == 1);
    CHECK(dual.statistics().rhsOnlySolves == masks.size());
  }
}

// Changed centers and repeated toggles match the rank-aware batch reference.
TEST(DenseQpSolver, CentersAndDependentToggles) {
  for (double weight : {0.0, .1, 1.0}) {
    const auto data = problem(weight);
    const BatchReference reference(data);
    DenseQpSolver updated(data);
    const std::vector<std::vector<bool>> masks{
        {false, false, false}, {true, false, false}, {true, true, true},
        {false, true, true}, {false, true, false}, {false, false, false},
        {true, true, false}, {false, true, false}};
    for (size_t i = 0; i < masks.size(); ++i) {
      const Vector center{{double(i) / 3, -double(i)}};
      const Vector actual = updated.solve(masks[i], center);
      EXPECT(assert_equal(reference.solve(masks[i], center), actual, 1e-9));
      EXPECT_DOUBLES_EQUAL(1, actual.sum(), 1e-10);
      EXPECT(assert_equal(reference.solve(masks[i], center * .5),
                          updated.solve(masks[i], center * .5), 1e-9));
    }
    EXPECT(updated.statistics().rebuildReasons.count(
        "basis_release_with_redundant_rows") > 0);
  }
}

// RHS-only queries perform zero numerical factorizations, including fallback.
TEST(DenseQpSolver, RhsOnlyUsesCachedFactors) {
  for (double weight : {0.0, .2}) {
    auto data = problem(weight);
    DenseQpSolver solver(data);
    const std::vector<bool> active{true, true, true};
    solver.solve(active, Vector::Zero(2));
    const size_t before = solver.statistics().numericalFactorizations;
    for (size_t i = 0; i < 5; ++i)
      solver.solve(active, Vector{{double(i), -double(i)}});
    EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  }
  DenseQpData data;
  data.jacobian = Matrix{{1, 0}, {0, 0}};
  data.rhs = Vector{{2, 0}};
  data.linear = Vector::Zero(2);
  data.normals = Matrix{{0, 1}};
  data.bounds = Vector::Ones(1);
  data.equalities = 1;
  DenseQpSolver singular(data);
  EXPECT(assert_equal(Vector{{2, 1}}, singular.solve({}, Vector::Zero(2)), 1e-10));
  const size_t before = singular.statistics().numericalFactorizations;
  singular.solve({}, Vector::Ones(2));
  EXPECT_LONGS_EQUAL(before, singular.statistics().numericalFactorizations);
}

// Independent additions and removals update triangular factors directly.
TEST(DenseQpSolver, IndependentRowsUseUpdates) {
  auto data = problem(.2);
  DenseQpSolver solver(data);
  solver.solve({false, false, false}, Vector::Zero(2));
  const size_t before = solver.statistics().numericalFactorizations;
  solver.solve({true, false, false}, Vector::Zero(2));
  solver.solve({false, false, false}, Vector::Zero(2));
  EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  EXPECT_LONGS_EQUAL(1, solver.statistics().rowAdditions);
  EXPECT_LONGS_EQUAL(1, solver.statistics().rowRemovals);
}

// Null-space QR retains weak weighted directions alongside exact boundaries.
TEST(DenseQpSolver, WeakDirectionAndAffineEquality) {
  DenseQpData data;
  data.jacobian = Matrix{{1, 0}, {0, 1e-10}};
  data.rhs = Vector{{0, 2e-10}};
  data.linear = Vector::Zero(2);
  data.normals = Matrix{{1, 0}, {2, 0}};
  data.bounds = Vector{{1, 2}};
  data.equalities = 2;
  for (double strong : {1.0, 1e10}) {
    data.jacobian(0, 0) = strong;
    const BatchReference reference(data);
    DenseQpSolver solver(data);
    EXPECT(assert_equal(Vector{{1, 2}}, reference.solve({}, Vector::Zero(2)), 1e-10));
    EXPECT(assert_equal(Vector{{1, 2}}, solver.solve({}, Vector::Zero(2)), 1e-10));
  }
}

// More active rows than variables must retain every original boundary.
TEST(DenseQpSolver, EightHundredThirtyTwoRows) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(384, 384);
  data.rhs = Vector::Zero(384);
  data.linear = Vector::Zero(384);
  data.normals = Matrix::Zero(832, 384);
  data.bounds = Vector::Ones(832);
  for (size_t i = 0; i < 832; ++i) data.normals(i, i % 384) = 1;
  DenseQpSolver solver(data);
  const Vector result = solver.solve(std::vector<bool>(832, true), Vector::Zero(384));
  EXPECT(assert_equal(Vector::Ones(384), result, 1e-10));
  EXPECT(assert_equal(data.bounds, Vector(data.normals * result), 1e-10));
}

// Removing an interior basis column updates all trailing LDLT factors correctly.
TEST(DenseQpSolver, InteriorBasisRemoval) {
  DenseQpData data;
  data.jacobian = Matrix{{2, 1, 0, 0}, {0, 3, 1, 0},
                         {0, 0, 4, 1}, {1, 0, 0, 2}};
  data.rhs = Vector{{1, 2, 3, 4}};
  data.linear = Vector::Zero(4);
  data.normals = Matrix::Identity(4, 4);
  data.bounds = Vector{{.1, -.2, .3, -.4}};
  data.regularization = .1;
  const BatchReference reference(data);
  DenseQpSolver updated(data);
  for (const std::vector<bool>& mask : std::vector<std::vector<bool>>{
       {true, true, true, true}, {true, false, true, true},
       {true, false, false, true}, {false, false, false, true},
       {true, true, true, true}, {false, true, true, true}}) {
    EXPECT(assert_equal(reference.solve(mask, Vector::Ones(4)),
                        updated.solve(mask, Vector::Ones(4)), 1e-10));
  }
}

// Removing several independent rows must not invent a redundant-row rebuild.
TEST(DenseQpSolver, BulkIndependentRemovalUsesUpdates) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(4, 4);
  data.rhs = Vector::Ones(4);
  data.linear = Vector::Zero(4);
  data.normals = Matrix::Identity(4, 4);
  data.bounds = Vector::Zero(4);
  DenseQpSolver solver(data);
  solver.solve({true, true, true, true}, Vector::Zero(4));
  const size_t before = solver.statistics().numericalFactorizations;
  EXPECT(assert_equal(Vector{{1, 1, 1, 0}},
                      solver.solve({false, false, false, true}, Vector::Zero(4)), 1e-10));
  EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  EXPECT_LONGS_EQUAL(3, solver.statistics().rowRemovals);
}

// Tiny nonzero linear forcing along a flat free direction is still unbounded.
TEST(DenseQpSolver, FlatLinearCostIsUnbounded) {
  DenseQpData data;
  data.jacobian = Matrix::Zero(2, 2);
  data.rhs = Vector::Zero(2);
  data.linear = Vector{{1e-20, 0}};
  data.normals = Matrix(0, 2);
  data.bounds = Vector(0);
  const BatchReference reference(data);
  DenseQpSolver solver(data);
  CHECK_EXCEPTION(reference.solve({}, Vector::Zero(2)), std::runtime_error);
  CHECK_EXCEPTION(solver.solve({}, Vector::Zero(2)), std::runtime_error);
}

// Mutually inconsistent affine boundaries are rejected rather than dropped.
TEST(DenseQpSolver, InconsistentBoundaries) {
  auto data = problem(0);
  data.bounds(2) = 2;
  DenseQpSolver solver(data);
  CHECK_EXCEPTION(solver.solve({true, true, false}, Vector::Zero(2)), std::runtime_error);
}

// Strong irreducible residuals must not contaminate an independent weak RHS.
TEST(DenseQpSolver, WeakRhsRefinement) {
  for (size_t dimension : {size_t(2), size_t(3)}) {
    DenseQpData data;
    data.jacobian = Matrix::Zero(3, dimension);
    data.jacobian(0, 0) = data.jacobian(1, 0) = 1;
    data.jacobian(2, 1) = 1e-10;
    data.rhs = Vector{{1, 2, 4e-10}};
    data.linear = Vector::Zero(dimension);
    data.normals.resize(0, dimension);
    data.bounds.resize(0);
    DenseQpSolver solver(data);
    Vector expected = Vector::Zero(dimension);
    expected.head(2) = Vector{{1.5, 4}};
    EXPECT(assert_equal(expected, solver.solve({}, Vector::Zero(dimension)), 1e-12));
    const size_t before = solver.statistics().numericalFactorizations;
    EXPECT(assert_equal(expected, solver.solve({}, Vector::Ones(dimension)), 1e-12));
    EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  }
}

// Refinement also preserves weak information supplied as a native linear term.
TEST(DenseQpSolver, WeakLinearForcingRefinement) {
  DenseQpData data;
  data.jacobian = Matrix{{1, 0}, {1, 0}, {0, 1e-10}};
  data.rhs = Vector{{1, 2, 0}};
  data.linear = Vector{{0, 4e-20}};
  data.normals.resize(0, 2);
  data.bounds.resize(0);
  DenseQpSolver solver(data);
  EXPECT(assert_equal(Vector{{1.5, 4}}, solver.solve({}, Vector::Zero(2)), 1e-12));
}

// Fixed coordinates stay exact as dependent affine boundaries enter and leave.
TEST(DenseQpSolver, FixedAndAffineEqualitiesPreserveWeakDirection) {
  DenseQpData data;
  data.jacobian = Vector4(1e10, 2, 3, 1e-10).asDiagonal();
  data.rhs = Vector{{0, 1, 2, 4e-10}};
  data.linear = Vector::Zero(4);
  data.normals = Matrix{{1, 0, 0, 0}, {2, 0, 0, 0}, {0, 1, 1, 0},
                        {5, 0, 0, 0}, {0, 1, 0, 0}, {0, -1, 0, 0}};
  data.bounds = Vector{{1, 2, 3, 5, .5, -.5}};
  data.equalities = 3;
  DenseQpSolver solver(data);
  for (const auto& active : std::vector<std::vector<bool>>{
           {true, false, false}, {true, true, true},
           {true, false, true}, {true, false, false}}) {
    const double second = active[1] || active[2] ? .5 : 23.0 / 13;
    const Vector expected{{1, second, 3 - second, 4}};
    EXPECT(assert_equal(expected, solver.solve(active, Vector::Zero(4)), 1e-12));
    const size_t before = solver.statistics().numericalFactorizations;
    EXPECT(assert_equal(expected, solver.solve(active, Vector::Ones(4)), 1e-12));
    EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  }
}

// A boundary identical to a permanent affine equality adds no free restriction.
TEST(DenseQpSolver, InequalityDependentOnAffineEquality) {
  DenseQpData data;
  data.jacobian = Matrix{{1, 0}, {0, 2}};
  data.rhs = data.linear = Vector::Zero(2);
  data.normals = Matrix{{1, 1}, {2, 2}, {-3, -3}};
  data.bounds = Vector{{1, 2, -3}};
  data.equalities = 1;
  DenseQpSolver solver(data);
  for (const auto& active : std::vector<std::vector<bool>>{
           {false, false}, {true, false}, {true, true}, {false, true}})
    EXPECT(assert_equal(Vector{{.8, .2}}, solver.solve(active, Vector::Zero(2)), 1e-12));
}

// A fully fixed graph needs no numerical objective or working factorization.
TEST(DenseQpSolver, FullEqualityRankAndInconsistentFixedRows) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(2, 2);
  data.rhs = data.linear = Vector::Zero(2);
  data.normals = Matrix{{1, 0}, {0, -2}, {3, 0}, {1, 0}};
  data.bounds = Vector{{2, 2, 6, 2}};
  data.equalities = 3;
  DenseQpSolver solver(data);
  EXPECT(assert_equal(Vector{{2, -1}}, solver.solve({true}, Vector::Ones(2)), 1e-12));
  EXPECT_LONGS_EQUAL(0, solver.statistics().objectiveFactorizations);
  const size_t before = solver.statistics().numericalFactorizations;
  solver.solve({false}, Vector::Zero(2));
  EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  data.bounds(2) = 9;
  CHECK_EXCEPTION((void)DenseQpSolver{data}, std::runtime_error);
}

// Feasibility and tangent users retain the same equality preparation across solves.
TEST(DenseQpSolver, EqualityPreparationIsStableAcrossWorkingChanges) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(3, 3);
  data.rhs = Vector{{2, 3, 4}};
  data.linear = Vector::Zero(3);
  data.normals = Matrix{{1, 1, 0}, {0, 0, 1}, {1, 0, 0}};
  data.bounds = Vector{{1, .25, 0}};
  data.equalities = 2;
  DenseQpSolver solver(data);
  const auto* prepared = solver.equalityProjection();
  EXPECT(prepared != nullptr);
  if (!prepared) return;
  EXPECT(assert_equal(Vector{{.5, .5, .25}}, prepared->point, 1e-12));
  const Matrix nullspace = prepared->nullspace;
  for (bool active : {false, true, false}) {
    solver.solve({active}, Vector::Zero(3));
    EXPECT(solver.equalityProjection() == prepared);
    EXPECT(assert_equal(nullspace, prepared->nullspace, 0));
    EXPECT(assert_equal(Vector::Zero(2),
        Vector(data.normals.topRows(2) * prepared->nullspace.col(0)), 1e-12));
  }
}

// Empty equality rows need no QR, but inconsistent original rows still fail.
TEST(DenseQpSolver, ZeroEqualitiesKeepFreeCoordinatesAndOriginalChecks) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(2, 2);
  data.rhs = Vector{{2, 3}};
  data.linear = Vector::Zero(2);
  data.normals = Matrix::Zero(2, 2);
  data.bounds = Vector::Zero(2);
  data.equalities = 2;
  DenseQpSolver solver(data);
  EXPECT(assert_equal(data.rhs, solver.solve({}, Vector::Zero(2)), 1e-12));
  const auto multipliers = solver.multipliers(data.rhs, Vector::Zero(2), 1e-9);
  EXPECT(multipliers.has_value());
  if (multipliers) EXPECT(assert_equal(Vector::Zero(2), *multipliers, 1e-12));
  EXPECT_LONGS_EQUAL(1, solver.statistics().numericalFactorizations);
  data.bounds(1) = 1;
  CHECK_EXCEPTION((void)DenseQpSolver{data}, std::runtime_error);
}

// Compacted equalities keep original multiplier indices and tiny nonzero rows.
TEST(DenseQpSolver, CompactedEqualitiesPreserveRowOriginsAndWeakRows) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(4, 4);
  data.rhs = Vector{{1, 2, 3, 4}};
  data.linear = Vector::Zero(4);
  data.normals = Matrix{{1, 0, 0, 0}, {0, 0, 0, 0}, {0, 2, 2, 0},
                        {3, 0, 0, 0}, {0, 0, 1e-10, 1e-10}};
  data.bounds = Vector{{2, 0, 6, 6, 5e-10}};
  data.equalities = 5;
  DenseQpSolver solver(data);
  const BatchReference reference(data);
  const Vector value = solver.solve({}, Vector::Zero(4));
  EXPECT(assert_equal(reference.solve({}, Vector::Zero(4)), value, 1e-10));
  const Vector gradient = value - data.rhs;
  const size_t before = solver.statistics().numericalFactorizations;
  const auto multipliers = solver.multipliers(value, gradient, 1e-9);
  EXPECT(multipliers.has_value());
  if (multipliers) {
    EXPECT_DOUBLES_EQUAL(0, (*multipliers)(1), 0);
    EXPECT(assert_equal(gradient, Vector(data.normals.transpose() * *multipliers),
                        1e-12));
  }
  solver.solve({}, Vector::Ones(4));
  EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
}

// Row scaling, signs, and updates reuse the working LDLT without factorization.
TEST(DenseQpSolver, CachedMultipliersFollowIndependentRows) {
  DenseQpData data;
  data.jacobian = Matrix{{2, 0}, {0, 3}};
  data.rhs = Vector{{4, -3}};
  data.linear = Vector::Zero(2);
  data.normals = Matrix{{2, 0}, {0, -3}, {2, 2}};
  data.bounds = Vector{{2, 0, 2}};
  DenseQpSolver solver(data);
  EXPECT(!solver.canRecoverMultipliers());
  const std::vector<std::vector<bool>> masks{
      {true, true, false}, {true, false, false},
      {false, false, true}};
  const std::vector<Vector> expected{Vector{{-2, -3, 0}}, Vector{{-2, 0, 0}},
                                     Vector::Zero(3)};
  for (size_t i = 0; i < masks.size(); ++i) {
    const Vector value = solver.solve(masks[i], Vector::Zero(2));
    const Vector gradient = data.jacobian.transpose() *
                            (data.jacobian * value - data.rhs);
    const size_t before = solver.statistics().numericalFactorizations;
    EXPECT(solver.canRecoverMultipliers());
    const auto multipliers = solver.multipliers(value, gradient, 1e-9);
    EXPECT(multipliers.has_value());
    if (multipliers) {
      EXPECT(assert_equal(expected[i], *multipliers, 1e-12));
      EXPECT(assert_equal(gradient, Vector(data.normals.transpose() * *multipliers),
                          1e-12));
    }
    EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  }
  // No active rows means the existing empty dual graph needs no gradient work.
  solver.solve({false, false, false}, Vector::Zero(2));
  EXPECT(!solver.canRecoverMultipliers());
  CHECK_EXCEPTION(solver.multipliers(Vector::Zero(2), Vector::Zero(3), 1e-9),
                  std::invalid_argument);
  EXPECT(!solver.multipliers(Vector::Zero(2), Vector::Constant(2,
      std::numeric_limits<double>::quiet_NaN()), 1e-9));
}

// Original stationarity must hold even when the proximal subproblem is solved.
TEST(DenseQpSolver, CachedMultipliersRejectProximalOnlyStationarity) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(2, 2);
  data.rhs = Vector{{2, 3}};
  data.linear = Vector::Zero(2);
  data.normals = Matrix{{1, 0}};
  data.bounds = Vector::Ones(1);
  data.regularization = .1;
  DenseQpSolver solver(data);
  const Vector value = solver.solve({true}, Vector::Zero(2));
  const Vector original = value - data.rhs;
  const size_t before = solver.statistics().numericalFactorizations;
  EXPECT(solver.canRecoverMultipliers());
  EXPECT(!solver.multipliers(value, original, 1e-9));
  const auto proximal = solver.multipliers(value, original + .1 * value, 1e-9);
  EXPECT(proximal.has_value());
  if (proximal) EXPECT(assert_equal(Vector{{-.9}}, *proximal, 1e-12));
  EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
}

// Small proximal stationarity error can still spoil an absolute original gap.
TEST(DenseQpSolver, CachedMultipliersRejectProximalGap) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(1, 1);
  data.rhs = Vector{{9999}};
  data.linear = Vector::Zero(1);
  data.normals = Matrix::Identity(1, 1);
  data.bounds = Vector{{10000}};
  data.regularization = 5e-14;
  DenseQpSolver solver(data);
  const Vector value = solver.solve({true}, Vector::Zero(1));
  const size_t before = solver.statistics().numericalFactorizations;
  EXPECT(solver.canRecoverMultipliers());
  EXPECT(!solver.multipliers(value, value - data.rhs, 1e-9));
  EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
}

// A rounded Gram pivot must not erase a real independent weak constraint.
TEST(DenseQpSolver, WeakIndependentSchurPivot) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(2, 2);
  data.rhs = data.linear = Vector::Zero(2);
  data.normals = Matrix{{1, 0}, {1, 1e-10}};
  data.bounds = Vector{{0, 2e-10}};
  DenseQpSolver solver(data);
  const BatchReference reference(data);
  for (const auto& active : std::vector<std::vector<bool>>{
           {true, true}, {false, true}, {true, true}, {true, false}}) {
    EXPECT(assert_equal(reference.solve(active, Vector::Zero(2)),
                        solver.solve(active, Vector::Zero(2)), 1e-10));
    const size_t before = solver.statistics().numericalFactorizations;
    solver.solve(active, Vector::Ones(2));
    EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
  }
  EXPECT_LONGS_EQUAL(0, solver.statistics().rebuildReasons.count("ambiguous_ldlt_pivot"));
}

// Many interior deletions and dependent-row exchanges retain the batch minimum.
TEST(DenseQpSolver, RepeatedLdltUpdates) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(8, 8);
  for (size_t i = 0; i < 7; ++i) data.jacobian(i, i + 1) = .3;
  data.rhs = Vector::LinSpaced(8, -1, 1);
  data.linear = Vector::Zero(8);
  data.normals = Matrix::Zero(11, 8);
  data.normals.topRows(8).setIdentity();
  data.normals.row(8) = 2 * data.normals.row(3);
  data.normals.row(9) = -data.normals.row(4);
  data.normals.row(10) = data.normals.row(1) + data.normals.row(7);
  data.bounds = data.normals * Vector::LinSpaced(8, -.2, .3);
  data.regularization = .1;
  DenseQpSolver solver(data);
  const BatchReference reference(data);
  for (size_t i = 0; i < 40; ++i) {
    std::vector<bool> active(11);
    for (size_t j = 0; j < 11; ++j) active[j] = (i * 3 + j * 5) % 7 < 4;
    const Vector center = Vector::Constant(8, .01 * i);
    EXPECT(assert_equal(reference.solve(active, center),
                        solver.solve(active, center), 1e-10));
  }
}

// Equality elimination supplies its existing QR to recover original row duals.
TEST(DenseQpSolver, CachedEqualityMultipliers) {
  auto data = problem(0);
  DenseQpSolver solver(data);
  const Vector value = solver.solve({false, false, false}, Vector::Zero(2));
  const Vector gradient = data.jacobian.transpose() *
                          (data.jacobian * value - data.rhs);
  const size_t before = solver.statistics().numericalFactorizations;
  EXPECT(solver.canRecoverMultipliers());
  const auto duals = solver.multipliers(value, gradient, 1e-9);
  EXPECT(duals.has_value());
  if (duals) EXPECT(assert_equal(Vector{{30.0 / 13, 0, 0, 0}}, *duals, 1e-10));
  EXPECT_LONGS_EQUAL(before, solver.statistics().numericalFactorizations);
}

// Fixed coordinates and equality-redundant inequalities retain original signs.
TEST(DenseQpSolver, CachedFixedCoordinateMultipliers) {
  DenseQpData data;
  data.jacobian = Matrix{{2, 0}, {0, 3}};
  data.rhs = Vector{{1, -1}};
  data.linear = Vector::Zero(2);
  data.normals = Matrix{{1, 0}, {1, 1}, {2, 0}, {-1, 0}};
  data.bounds = Vector{{1, 3, 2, -1}};
  data.equalities = 2;
  DenseQpSolver solver(data);
  const Vector value = solver.solve({true, true}, Vector::Zero(2));
  const Vector gradient = data.jacobian.transpose() *
                          (data.jacobian * value - data.rhs);
  EXPECT(solver.canRecoverMultipliers());
  const auto duals = solver.multipliers(value, gradient, 1e-9);
  EXPECT(duals.has_value());
  if (duals) EXPECT(assert_equal(Vector{{-19, 21, 0, 0}}, *duals, 1e-10));
}

// Dependent inequalities may reuse a valid gauge; invalid signs retain recovery.
TEST(DenseQpSolver, CachedMultipliersRetainDifficultSystemFallbacks) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(1, 1);
  data.rhs = Vector{{2}};
  data.linear = Vector::Zero(1);
  data.normals = Matrix{{1}, {2}, {-1}};
  data.bounds = Vector{{1, 2, -1}};
  DenseQpSolver dependent(data);
  dependent.solve({true, true, true}, Vector::Zero(1));
  EXPECT(dependent.canRecoverMultipliers());
  const auto duals = dependent.multipliers(Vector{{1}}, Vector{{-1}}, 1e-9);
  EXPECT(duals.has_value());
  if (duals) EXPECT(assert_equal(Vector{{-1, 0, 0}}, *duals, 1e-10));
  data.rhs = Vector::Zero(1);
  DenseQpSolver alternativeGauge(data);
  alternativeGauge.solve({true, true, true}, Vector::Zero(1));
  EXPECT(!alternativeGauge.multipliers(Vector{{1}}, Vector{{1}}, 1e-9));
}

// A weak weighted objective still yields its small, correctly signed multiplier.
TEST(DenseQpSolver, CachedMultipliersPreserveWeakGradient) {
  DenseQpData data;
  data.jacobian = Matrix{{1, 0}, {0, 1e-10}};
  data.rhs = data.linear = Vector::Zero(2);
  data.normals = Matrix{{0, 1}};
  data.bounds = Vector{{2}};
  DenseQpSolver solver(data);
  const Vector value = solver.solve({true}, Vector::Zero(2));
  const Vector gradient = data.jacobian.transpose() * data.jacobian * value;
  const auto multipliers = solver.multipliers(value, gradient, 1e-9);
  EXPECT(multipliers.has_value());
  if (multipliers) EXPECT(assert_equal(Vector{{2e-20}}, *multipliers, 1e-29));
}

// Equality multipliers are unrestricted even when dependent working rows
// trigger the conservative gauge check in original coordinates.
TEST(DenseQpSolver, UnprojectedMultiplierGauge) {
  DenseQpData data;
  data.jacobian = Matrix::Identity(3, 3);
  data.rhs = Vector3{-1, -1, 2};
  data.linear = Vector::Zero(3);
  data.normals = Matrix{{1, 1, 0}, {2, 2, 0}, {0, 0, 1}};
  data.bounds = Vector3{1, 2, 0};
  data.equalities = 2;
  DenseQpSolver solver(data, false);
  const Vector value = solver.solve({true}, Vector::Zero(3));
  EXPECT(assert_equal(Vector3(.5, .5, 0), value, 1e-10));
  const auto multipliers = solver.multipliers(value, value - data.rhs, 1e-9);
  CHECK(multipliers.has_value());
  if (multipliers) {
    EXPECT(assert_equal(Vector(value - data.rhs),
                        Vector(data.normals.transpose() * *multipliers), 1e-10));
    DOUBLES_EQUAL(-2, (*multipliers)(2), 1e-10);
  }
}

// Unprojected dual responses preserve exact equalities and reuse factors for
// arbitrary forcing and proximal centers, including a globally singular cost.
TEST(DenseQpSolver, ConstrainedDualWithoutProjection) {
  for (double regularization : {0.0, 0.1}) {
    DenseQpData data;
    data.jacobian = Matrix{{1, 0, 0}, {0, 1, 0}};
    data.rhs = Vector2{3, 2};
    data.linear = Vector3{0, 0, 0};
    data.normals = Matrix{{1, 1, 1}, {2, 2, 2}, {1, 0, 0}};
    data.bounds = Vector3{1, 2, 0};
    data.equalities = 2;
    data.regularization = regularization;
    DenseQpSolver direct(data, false), projected(data, true);
    CHECK(direct.equalityProjection() == nullptr);
    CHECK(direct.reducedDimension() == 2);
    CHECK(direct.prepareDual());
    CHECK(projected.prepareDual());
    const auto factorizations = direct.statistics().numericalFactorizations;
    for (const Vector3& center : {Vector3{0, 0, 0}, Vector3{1, -2, 9}}) {
      const Vector3 forcing{2, -3, 4};
      EXPECT(assert_equal(projected.solveDual(center, forcing),
                          direct.solveDual(center, forcing), 1e-9));
      const Vector response = direct.dualResponse(forcing);
      EXPECT(assert_equal(projected.dualResponse(forcing), response, 1e-9));
      EXPECT(assert_equal(Vector::Zero(2),
                          Vector(data.normals.topRows(2) * response), 1e-10));
    }
    CHECK(direct.statistics().numericalFactorizations == factorizations);
    CHECK(direct.statistics().objectiveFactorizations == 1);
    CHECK(direct.statistics().rhsOnlySolves == 2);
  }
}

}  // namespace dense_working_system
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
