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
 * This file isolates square-root objective factors, working LDLT updates,
 * and cached RHS solves from the shared active-set iteration. Its cached
 * null-space solver handles singular objective curvature and difficult working
 * constraints while retaining checks on every selected original row.
 *
 * The private implementation keeps Eigen factorization details out of the
 * interface header. Mutable numerical caches belong to one optimization call;
 * the uncached batch reference is defined in testDenseQpSolver.cpp.
 */

#include <gtsam/constrained/internal/DenseQpSolver.h>
#include <gtsam/constrained/internal/DenseQpEqualityProjection.h>

#include <Eigen/QR>
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
  const DenseQpData& original;
  std::optional<DenseQpEqualityProjection> projection;
  const DenseQpData& data;
  Matrix objective;
  bool objectiveReady = false, invertible = false, workingReady = false;
  Eigen::ColPivHouseholderQR<Matrix> objectiveQr;
  Matrix objectiveR, workingNormals, workingL;
  Vector workingD, workingMultipliers;
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


  explicit Impl(const DenseQpData& source)
      : original(source), projection(projectDenseQpEqualities(source)),
        data(projection ? projection->reduced : source) {
    if (projection)
      statistics.numericalFactorizations += projection->numericalFactorizations;
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

  // Solve the PSD Schur system from its unit triangular factors and pivots.
  Vector solveWorking(const Vector& rhs) const {
    Vector result = workingL.triangularView<Eigen::UnitLower>().solve(rhs);
    result.array() /= workingD.array();
    return workingL.transpose().triangularView<Eigen::UnitUpper>().solve(result);
  }

  // Border LDLT as in Arnstrom et al., IEEE TAC 2022, equations (14)-(15).
  // Square-root residuals distinguish dependent rows from weak D pivots.
  bool add(size_t row, bool countUpdate = true) {
    const Vector& column = normal(row);
    Vector lower = workingL.triangularView<Eigen::UnitLower>().solve(
        Vector(workingNormals.transpose() * column));
    lower.array() /= workingD.array();
    Vector remainder = column - workingNormals * workingL.transpose()
        .triangularView<Eigen::UnitUpper>().solve(lower);
    // Compute D from the square-root residual rather than subtract nearly
    // equal Gram terms. A second projection preserves weak independent rows.
    Vector correction = workingL.triangularView<Eigen::UnitLower>().solve(
        Vector(workingNormals.transpose() * remainder));
    correction.array() /= workingD.array();
    lower += correction;
    remainder -= workingNormals * workingL.transpose()
        .triangularView<Eigen::UnitUpper>().solve(correction);
    const double length = remainder.stableNorm();
    const double tolerance = 2 * std::numeric_limits<double>::epsilon() *
                             data.linear.size();
    if (length <= tolerance) { ++statistics.dependentRows; return true; }
    const double pivot = length * length;
    if (!std::isfinite(pivot) || basis.size() >= size_t(data.linear.size()))
      return false;
    const Eigen::Index rank = basis.size();
    workingL.conservativeResize(rank + 1, rank + 1);
    workingL.col(rank).head(rank).setZero();
    workingL.row(rank).head(rank) = lower.transpose();
    workingL(rank, rank) = 1;
    workingD.conservativeResize(rank + 1);
    workingD(rank) = pivot;
    workingNormals.conservativeResize(Eigen::NoChange, rank + 1);
    workingNormals.col(rank) = column;
    basis.push_back(row);
    if (countUpdate) ++statistics.rowAdditions;
    return true;
  }

  // Build a row-ordered independent basis. Every original selected row remains
  // in activeRows and is checked even when its multiplier gauge is redundant.
  void rebuild(const std::vector<size_t>& rows, const std::string& reason) {
    ++statistics.rebuildReasons[reason];
    ++statistics.workingRebuilds;
    basis.clear();
    workingNormals.resize(data.linear.size(), 0);
    workingL.resize(0, 0); workingD.resize(0);
    if (!rows.empty()) ++statistics.numericalFactorizations;
    for (size_t row : rows) {
      if (add(row, false)) continue;
      buildNullspaceSystem(rows, "ambiguous_ldlt_pivot");
      break;
    }
  }

  // Deletion is a positive rank-one update of the trailing LDLT factors:
  // Arnstrom et al., equations (16)-(17). No matrix inverse or rebuild occurs.
  void remove(size_t position) {
    const Eigen::Index rank = basis.size(), remaining = rank - position - 1;
    Vector update = workingL.col(position).tail(remaining);
    double weight = workingD(position);
    Matrix lower = Matrix::Identity(rank - 1, rank - 1);
    lower.topLeftCorner(position, position) = workingL.topLeftCorner(position, position);
    lower.bottomLeftCorner(remaining, position) = workingL.bottomLeftCorner(remaining, position);
    lower.bottomRightCorner(remaining, remaining) = workingL.bottomRightCorner(remaining, remaining);
    Vector diagonal(rank - 1);
    diagonal.head(position) = workingD.head(position);
    diagonal.tail(remaining) = workingD.tail(remaining);
    for (Eigen::Index i = 0; i < remaining; ++i) {
      const Eigen::Index index = position + i, trailing = remaining - i - 1;
      const double previous = diagonal(index), component = update(i);
      diagonal(index) += weight * component * component;
      const double coefficient = weight * component / diagonal(index);
      weight *= previous / diagonal(index);
      update.tail(trailing) -= component * lower.col(index).tail(trailing);
      lower.col(index).tail(trailing) += coefficient * update.tail(trailing);
    }
    workingL = std::move(lower); workingD = std::move(diagonal);
    workingNormals.middleCols(position, remaining) =
        workingNormals.rightCols(remaining).eval();
    workingNormals.conservativeResize(Eigen::NoChange, rank - 1);
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
      else for (size_t row : added) {
        if (add(row)) continue;
        buildNullspaceSystem(rows, "ambiguous_ldlt_pivot");
        break;
      }
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
    Vector multipliers = solveWorking(rhs);
    Vector value = target + workingNormals * multipliers;
    Vector result;
    for (size_t attempt = 0; attempt < 3; ++attempt) {
      result = objectiveQr.colsPermutation() *
          objectiveR.triangularView<Eigen::Upper>().solve(value);
      if (attempt == 2) break;
      for (size_t i = 0; i < basis.size(); ++i)
        rhs(i) = (data.bounds(basis[i]) - data.normals.row(basis[i]).dot(result)) /
                 scales[basis[i]];
      const Vector correction = solveWorking(rhs);
      multipliers += correction;
      value += workingNormals * correction;
    }
    workingMultipliers = Vector::Zero(data.bounds.size());
    for (size_t i = 0; i < basis.size(); ++i)
      workingMultipliers(basis[i]) = multipliers(i) / scales[basis[i]];
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
    if (changed) {
      if (nullspaceCache) workingReady = false;
      nullspaceCache.reset();
    } else ++statistics.rhsOnlySolves;
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


  // Working factors can propose a multiplier gauge. Original stationarity,
  // gap, and (for dependent inequalities) signs decide whether it is usable.
  bool canRecoverMultipliers() const {
    if (!workingReady || nullspaceCache || (!invertible && data.linear.size()))
      return false;
    if (!projection && (original.equalities || basis.empty())) return false;
    return true;
  }

  // The primal LDLT supplied inequality multipliers; the retained equality QR
  // supplies the rest. Original-gradient checks require no new factorization.
  std::optional<Vector> multipliers(const Vector& value, const Vector& gradient,
                                    double tolerance) const {
    if (value.size() != original.linear.size() ||
        gradient.size() != original.linear.size())
      throw std::invalid_argument("Dense QP: invalid multiplier gradient size.");
    if (!canRecoverMultipliers() || !gradient.allFinite()) return std::nullopt;
    Vector result = Vector::Zero(original.bounds.size());
    result.tail(workingMultipliers.size()) = workingMultipliers;
    if (projection) {
      const Vector remaining = gradient -
          original.normals.bottomRows(workingMultipliers.size()).transpose() *
              workingMultipliers;
      result.head(original.equalities) = projection->multipliers(original, remaining);
    }
    // A nonunique gauge may certify optimality only with valid inequality
    // signs. Otherwise preserve graph recovery and its constraint-release rule.
    const size_t nonzero = std::count_if(activeRows.begin(), activeRows.end(),
        [&](size_t row) { return !data.normals.row(row).isZero(0); });
    if (nonzero > basis.size() &&
        (result.tail(data.bounds.size()).array() > tolerance).any())
      return std::nullopt;
    // Match the graph recovery's threshold; the original-QP KKT certificate
    // still makes the final decision using the original factors and values.
    const double residual = (original.normals.transpose() * result - gradient)
                                .lpNorm<Eigen::Infinity>();
    const double threshold = std::max(1e-12, tolerance) *
                             (1 + gradient.lpNorm<Eigen::Infinity>());
    if (!result.allFinite() || !std::isfinite(residual) || residual > threshold)
      return std::nullopt;
    // Proximal multipliers need not certify the original QP. Retain graph
    // recovery when their residual could spoil an acceptable absolute gap.
    const double gap = value.dot(gradient) - original.bounds.dot(result);
    if (!std::isfinite(gap) || std::abs(gap) > tolerance) return std::nullopt;
    return result;
  }


  // Project only the center/RHS during working-set exploration. A fully fixed
  // graph needs no objective or working-system factorization at all.
  Vector solveOriginal(const std::vector<bool>& active, const Vector& center) {
    const auto rows = selected(active);
    if (!projection) return solve(rows, center);
    Vector value = projection->point;
    if (data.linear.size() == 0) {
      ++statistics.solves;
      if (workingReady && rows == activeRows) ++statistics.rhsOnlySolves;
      activeRows = rows;
      workingReady = true;
      workingMultipliers = Vector::Zero(data.bounds.size());
    } else {
      const Vector reducedCenter = projection->nullspace.transpose() *
                                   (center - projection->point);
      value += projection->nullspace * solve(rows, reducedCenter);
    }
    checkDenseQpOriginalWorkingSet(original, value, active);
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
  return impl_->solveOriginal(active, center);
}

bool DenseQpSolver::canRecoverMultipliers() const {
  return impl_->canRecoverMultipliers();
}
std::optional<Vector> DenseQpSolver::multipliers(const Vector& value,
                                               const Vector& gradient,
                                               double tolerance) const {
  return impl_->multipliers(value, gradient, tolerance);
}

}  // namespace internal
}  // namespace gtsam
