/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DenseQpSolver.cpp
 * Production dense working-system algebra used by ActiveSetSolver.
 *
 * This file isolates packed dense QR factorizations, constraint-row updates,
 * and cached RHS solves from the shared active-set iteration. Its cached
 * null-space solver handles singular objective curvature and difficult working
 * constraints while retaining checks on every selected original row.
 *
 * The private implementation keeps Eigen factorization details out of the
 * interface header. Mutable numerical caches belong to one optimization call;
 * the uncached batch reference is defined in testDenseQpSolver.cpp.
 */

#include <gtsam/constrained/internal/DenseQpSolver.h>

#include <Eigen/QR>
#include <Eigen/Jacobi>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam {
namespace internal {

namespace {

// Q^T*rhs can mix an irreducible residual into a weak RHS. Correct the original
// square-root gradient using the cached QR basis, never a normal matrix. Static
// linear forcing has already been checked for consistency before refinement.
Vector refineQrMinimum(const Matrix& jacobian, const Vector& rhs,
                       const Vector& linear,
                       const Eigen::ColPivHouseholderQR<Matrix>& qr,
                       Vector value) {
  const Eigen::Index rank = qr.rank();
  for (size_t refinement = 0; refinement < 2; ++refinement) {
    const Vector gradient = jacobian.transpose() * (rhs - jacobian * value) + linear;
    const Vector ordered = qr.colsPermutation().transpose() * gradient;
    const Vector forward = qr.matrixR().topLeftCorner(rank, rank).transpose()
        .triangularView<Eigen::Lower>().solve(ordered.head(rank));
    Vector correction = Vector::Zero(qr.cols());
    correction.head(rank) = qr.matrixR().topLeftCorner(rank, rank)
        .triangularView<Eigen::Upper>().solve(forward);
    value += Vector(qr.colsPermutation() * correction);
  }
  return value;
}

}  // namespace

struct DenseQpSolver::Impl {
  const DenseQpData& data;
  Matrix objective;
  bool objectiveReady = false, invertible = false, workingReady = false;
  Eigen::ColPivHouseholderQR<Matrix> objectiveQr;
  Matrix objectiveR, directions, workingR;
  Vector unconstrainedTarget;
  std::vector<size_t> activeRows, basis;
  std::vector<Vector> transformed;
  std::vector<double> scales;
  Statistics statistics;

  struct NullspaceSystem {
    Vector point, correction;
    Matrix nullspace, jacobian;
    std::vector<Eigen::Index> varying;
    std::unique_ptr<Eigen::ColPivHouseholderQR<Matrix>> qr;
  };
  std::unique_ptr<NullspaceSystem> nullspaceCache;


  explicit Impl(const DenseQpData& source) : data(source) {
    // Append exactly the requested proximal curvature in square-root form.
    const Eigen::Index n = data.linear.size();
    const Eigen::Index extra = data.regularization > 0 ? n : 0;
    objective = Matrix::Zero(data.jacobian.rows() + extra, n);
    objective.topRows(data.jacobian.rows()) = data.jacobian;
    if (extra) objective.bottomRows(n).diagonal().setConstant(
        std::sqrt(data.regularization));
  }

  // All row identities survive numerical basis selection, including redundant
  // equalities and opposing inequalities that temporarily bind the same point.
  std::vector<size_t> selected(const std::vector<bool>& active) const {
    if (active.size() + data.equalities != size_t(data.bounds.size()))
      throw std::invalid_argument("Dense QP: invalid working-set mask.");
    std::vector<size_t> rows;
    for (size_t row = 0; row < data.equalities; ++row) rows.push_back(row);
    for (size_t row = 0; row < active.size(); ++row)
      if (active[row]) rows.push_back(data.equalities + row);
    return rows;
  }

  // Keep finite weights in their original square-root representation. The
  // proximal center changes this vector, never objective or exact constraints.
  Vector target(const Vector& center) const {
    Vector result(objective.rows());
    result.head(data.rhs.size()) = data.rhs;
    if (data.regularization > 0)
      result.tail(center.size()) = std::sqrt(data.regularization) * center;
    return result;
  }

  // Normalize rows only for rank detection and solving, without changing the
  // original rows used for feasibility checks and multiplier recovery.
  std::pair<Matrix, Vector> constraints(
      const std::vector<size_t>& rows) const {
    Matrix normals(rows.size(), data.normals.cols());
    Vector bounds(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
      const size_t row = rows[i];
      const double scale = data.normals.row(row).stableNorm();
      normals.row(i) = data.normals.row(row) / (scale > 0 ? scale : 1);
      bounds(i) = data.bounds(row) / (scale > 0 ? scale : 1);
    }
    return {normals, bounds};
  }

  // Rank-revealing QR identifies independent boundary equations. Their full
  // orthogonal complement parameterizes feasible motion without a Schur inverse.
  std::pair<Vector, Matrix> feasibleSpace(const Matrix& normals,
                                         const Vector& bounds) const {
    const Eigen::Index n = data.linear.size();
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

  // Check every selected row rather than assuming the independent basis also
  // represents consistent redundant affine boundaries.
  void check(const Vector& value, const std::vector<size_t>& rows) const {
    if (!value.allFinite())
      throw std::runtime_error("Dense QP: non-finite working-face solution.");
    for (size_t row : rows) {
      const double scale = 1 + std::abs(data.bounds(row)) +
          data.normals.row(row).cwiseAbs().dot(value.cwiseAbs());
      const double allowance = 256 * std::numeric_limits<double>::epsilon() * scale;
      if (std::abs(data.normals.row(row).dot(value) - data.bounds(row)) >
          std::max(1e-10, allowance))
        throw std::runtime_error("Dense QP: inconsistent working constraints.");
    }
  }

  // Curvature is fixed for the entire optimization call. QR preserves supplied
  // square roots; transformed normals are computed only when first needed.
  void prepareObjective() {
    objectiveQr.compute(objective);
    ++statistics.numericalFactorizations;
    ++statistics.objectiveFactorizations;
    invertible = objectiveQr.rank() == objective.cols();
    objectiveReady = true;
    transformed.resize(data.bounds.size());
    scales.resize(data.bounds.size());
    if (!invertible) return;
    const Eigen::Index n = objective.cols();
    objectiveR = objectiveQr.matrixR().topRows(n)
                     .triangularView<Eigen::Upper>();
    unconstrainedTarget = (objectiveQr.householderQ().adjoint() *
                           target(Vector::Zero(n))).head(n);
    unconstrainedTarget += objectiveR.transpose().triangularView<Eigen::Lower>()
        .solve(Vector(objectiveQr.colsPermutation().transpose() * data.linear));
    // Preserve weak RHS information before working constraints are imposed.
    const Vector value = objectiveQr.colsPermutation() *
        objectiveR.triangularView<Eigen::Upper>().solve(unconstrainedTarget);
    const Vector refined = refineQrMinimum(objective, target(Vector::Zero(n)),
                                           data.linear, objectiveQr, value);
    unconstrainedTarget = objectiveR *
        (objectiveQr.colsPermutation().transpose() * refined);
  }

  // In whitened coordinates curvature is the identity. No inverse matrix is
  // formed, and each original normal needs only one cached triangular solve.
  const Vector& normal(size_t row) {
    if (transformed[row].size() == 0) {
      Vector value = objectiveR.transpose().triangularView<Eigen::Lower>().solve(
          Vector(objectiveQr.colsPermutation().transpose() *
                 data.normals.row(row).transpose()));
      scales[row] = value.stableNorm();
      if (scales[row] == 0) scales[row] = 1;
      transformed[row] = value / scales[row];
      ++statistics.transformedNormals;
    }
    return transformed[row];
  }

  // A pivoted rebuild chooses a numerical basis while retaining every active
  // row identity for subsequent consistency checks and multiplier recovery.
  void rebuild(const std::vector<size_t>& rows, const std::string& reason) {
    ++statistics.rebuildReasons[reason];
    ++statistics.workingRebuilds;
    basis.clear();
    const Eigen::Index n = data.linear.size();
    if (rows.empty()) {
      directions.resize(n, 0); workingR.resize(0, 0); return;
    }
    Matrix columns(n, rows.size());
    for (size_t i = 0; i < rows.size(); ++i) columns.col(i) = normal(rows[i]);
    Eigen::ColPivHouseholderQR<Matrix> qr(columns);
    ++statistics.numericalFactorizations;
    const Eigen::Index rank = qr.rank();
    directions = qr.householderQ() * Matrix::Identity(n, rank);
    workingR = qr.matrixR().topLeftCorner(rank, rank)
                   .triangularView<Eigen::Upper>();
    for (Eigen::Index i = 0; i < rank; ++i)
      basis.push_back(rows[qr.colsPermutation().indices()(i)]);
    statistics.dependentRows += rows.size() - rank;
  }

  // Two orthogonal projections guard independent-row insertions against loss
  // of orthogonality. Dependent rows remain selected; they add no basis column.
  void add(size_t row) {
    const Vector& column = normal(row);
    Vector coefficients = directions.transpose() * column;
    Vector remainder = column - directions * coefficients;
    const Vector correction = directions.transpose() * remainder;
    coefficients += correction;
    remainder -= directions * correction;
    const double length = remainder.stableNorm();
    const double threshold = std::numeric_limits<double>::epsilon() *
                             data.linear.size() * 2;
    if (length <= threshold) { ++statistics.dependentRows; return; }
    const Eigen::Index rank = basis.size();
    workingR.conservativeResize(rank + 1, rank + 1);
    workingR.col(rank).head(rank) = coefficients;
    workingR.row(rank).head(rank).setZero();
    workingR(rank, rank) = length;
    directions.conservativeResize(Eigen::NoChange, rank + 1);
    directions.col(rank) = remainder / length;
    basis.push_back(row);
    ++statistics.rowAdditions;
  }

  // Deleting a basis column creates one subdiagonal. Givens rotations restore
  // triangular form and rotate the associated orthogonal directions in place.
  void remove(size_t position) {
    const Eigen::Index rank = basis.size();
    Matrix shifted(rank, rank - 1);
    shifted.leftCols(position) = workingR.leftCols(position);
    shifted.rightCols(rank - position - 1) = workingR.rightCols(rank - position - 1);
    for (Eigen::Index i = position; i + 1 < rank; ++i) {
      Eigen::JacobiRotation<double> rotation;
      rotation.makeGivens(shifted(i, i), shifted(i + 1, i));
      shifted.applyOnTheLeft(i, i + 1, rotation.adjoint());
      directions.applyOnTheRight(i, i + 1, rotation);
    }
    workingR = shifted.topRows(rank - 1);
    directions.conservativeResize(Eigen::NoChange, rank - 1);
    basis.erase(basis.begin() + position);
    ++statistics.rowRemovals;
  }

  // Releasing a basis row can make a redundant original row independent. A
  // controlled pivoted rebuild performs that exchange instead of dropping it.
  void update(const std::vector<size_t>& rows) {
    if (!workingReady) {
      rebuild(rows, "initial_working_set");
    } else {
      std::vector<size_t> removed, added;
      std::set_difference(activeRows.begin(), activeRows.end(), rows.begin(), rows.end(),
                          std::back_inserter(removed));
      std::set_difference(rows.begin(), rows.end(), activeRows.begin(), activeRows.end(),
                          std::back_inserter(added));
      const bool hadRedundantRows = activeRows.size() > basis.size();
      bool exchange = false;
      for (size_t row : removed) {
        const auto found = std::find(basis.begin(), basis.end(), row);
        if (found == basis.end()) continue;
        if (hadRedundantRows) { exchange = true; break; }
        remove(found - basis.begin());
      }
      if (exchange) rebuild(rows, "basis_release_with_redundant_rows");
      else for (size_t row : added) add(row);
    }
    activeRows = rows;
    workingReady = true;
  }

  // Original input coordinates remain authoritative. Refinement uses only the
  // cached triangular working system, including on RHS-only queries.
  Vector whitenedSolve(const Vector& center) {
    Vector target = unconstrainedTarget;
    if (data.regularization > 0)
      target += objectiveR.transpose().triangularView<Eigen::Lower>().solve(
          Vector(objectiveQr.colsPermutation().transpose() *
                 (data.regularization * center)));
    Vector rhs(basis.size());
    for (size_t i = 0; i < basis.size(); ++i)
      rhs(i) = data.bounds(basis[i]) / scales[basis[i]] - normal(basis[i]).dot(target);
    Vector value = target + directions *
        workingR.transpose().triangularView<Eigen::Lower>().solve(rhs);
    Vector result;
    for (size_t attempt = 0; attempt < 3; ++attempt) {
      result = objectiveQr.colsPermutation() *
          objectiveR.triangularView<Eigen::Upper>().solve(value);
      if (attempt == 2) break;
      for (size_t i = 0; i < basis.size(); ++i)
        rhs(i) = (data.bounds(basis[i]) - data.normals.row(basis[i]).dot(result)) /
                 scales[basis[i]];
      value += directions * workingR.transpose().triangularView<Eigen::Lower>().solve(rhs);
    }
    return result;
  }

  // The rank-aware fallback is cached too: neither singular curvature nor
  // exact constraints justify re-elimination when only the forcing changes.
  void buildNullspaceSystem(const std::vector<size_t>& rows, const std::string& reason) {
    ++statistics.rebuildReasons[reason];
    ++statistics.workingRebuilds;
    nullspaceCache = std::make_unique<NullspaceSystem>();
    auto& cache = *nullspaceCache;
    const auto system = constraints(rows);
    const auto space = feasibleSpace(system.first, system.second);
    if (!rows.empty()) ++statistics.numericalFactorizations;
    cache.point = space.first;
    cache.nullspace = space.second;
    if (cache.nullspace.cols() == 0) return;
    const Matrix full = objective * cache.nullspace;
    for (Eigen::Index row = 0; row < full.rows(); ++row)
      if (!full.row(row).isZero(0)) cache.varying.push_back(row);
    if (cache.varying.empty()) {
      if ((cache.nullspace.transpose() * data.linear).norm() > 0)
        throw std::runtime_error("Dense QP: unbounded working-face linear cost.");
      return;
    }
    Matrix reduced(cache.varying.size(), full.cols());
    for (size_t i = 0; i < cache.varying.size(); ++i)
      reduced.row(i) = full.row(cache.varying[i]);
    cache.jacobian = std::move(reduced);
    cache.qr = std::make_unique<Eigen::ColPivHouseholderQR<Matrix>>(cache.jacobian);
    ++statistics.numericalFactorizations;
    cache.correction = linearCorrection(*cache.qr,
                                        cache.nullspace.transpose() * data.linear);
  }

  // Full and rank-deficient reduced costs use the same forcing consistency
  // check; no normal equations, added curvature, or tolerance changes occur.
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

  // A new proximal center changes only reduced residuals and triangular RHSs.
  Vector solveNullspace(const Vector& center) const {
    const auto& cache = *nullspaceCache;
    if (!cache.qr) return cache.point;
    const Vector full = target(center) - objective * cache.point;
    Vector rhs(cache.varying.size());
    for (size_t i = 0; i < cache.varying.size(); ++i) rhs(i) = full(cache.varying[i]);
    const Vector minimum = refineQrMinimum(cache.jacobian, rhs,
        Vector(cache.nullspace.transpose() * data.linear), *cache.qr,
        Vector(cache.qr->solve(rhs) + cache.correction));
    return cache.point + cache.nullspace * minimum;
  }

  // An unchanged working set uses cached factors, including the null-space
  // fallback. Only selected-row changes can require numerical factorization.
  Vector solve(const std::vector<size_t>& rows, const Vector& center) {
    ++statistics.solves;
    if (!objectiveReady) prepareObjective();
    const bool changed = !workingReady || rows != activeRows;
    if (changed) nullspaceCache.reset();
    else ++statistics.rhsOnlySolves;
    if (!invertible) {
      if (changed) buildNullspaceSystem(rows, "singular_objective");
      activeRows = rows; workingReady = true;
    } else if (changed) {
      update(rows);
    }
    Vector value;
    if (nullspaceCache) value = solveNullspace(center);
    else {
      value = whitenedSolve(center);
      try { check(value, rows); }
      catch (const std::runtime_error&) {
        // A row change may trigger a controlled reference rebuild. An RHS-only
        // query must either solve with its caches or report numerical failure.
        if (!changed) throw;
        buildNullspaceSystem(rows, "constraint_refinement_failed");
        value = solveNullspace(center);
      }
    }
    check(value, rows);
    return value;
  }

};

DenseQpSolver::DenseQpSolver(const DenseQpData& data)
    : impl_(std::make_unique<Impl>(data)) {}
DenseQpSolver::~DenseQpSolver() = default;
const DenseQpSolver::Statistics& DenseQpSolver::statistics() const {
  return impl_->statistics;
}
Vector DenseQpSolver::solve(const std::vector<bool>& active, const Vector& center) {
  // Numerical state belongs to this call, never the owning factor graph.
  return impl_->solve(impl_->selected(active), center);
}

}  // namespace internal
}  // namespace gtsam
