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
#include <gtsam/linear/internal/ConstrainedQR.h>

#include <Eigen/QR>
#include <Eigen/Cholesky>
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
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> objectivePermutation;
  bool nativeCholesky = false;
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

  // Full-coordinate dual factors retain exact equality pivots. Their zero
  // variances suppress motion across equalities without constructing Z or H.
  Eigen::ColPivHouseholderQR<Matrix> equalityQr;
  Vector equalityScales, equalityPoint, dualPoint, dualVariances;
  Matrix dualR;
  bool equalitiesReady = false, constrainedDualReady = false;


  explicit Impl(const DenseQpData& source, bool projectEqualities)
      : original(source), projection(projectEqualities
            ? projectDenseQpEqualities(source) : std::nullopt),
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

  // Factor normalized E^T for rank, affine consistency and original-row
  // multiplier recovery. Only its range is used, never its nullspace.
  void prepareEqualities() {
    if (equalitiesReady) return;
    Matrix normals = data.normals.topRows(data.equalities);
    equalityScales.resize(data.equalities);
    for (size_t row = 0; row < data.equalities; ++row) {
      const double norm = normals.row(row).stableNorm();
      equalityScales(row) = norm > 0 ? norm : 1;
      normals.row(row) /= equalityScales(row);
    }
    equalityQr.compute(normals.transpose());
    ++statistics.numericalFactorizations;
    const Eigen::Index n = data.linear.size(), rank = equalityQr.rank();
    const Vector bounds = data.bounds.head(data.equalities).cwiseQuotient(equalityScales);
    const Vector ordered = equalityQr.colsPermutation().transpose() * bounds;
    Vector coordinates = Vector::Zero(n);
    coordinates.head(rank) = equalityQr.matrixR().topLeftCorner(rank, rank)
        .transpose().triangularView<Eigen::Lower>().solve(ordered.head(rank));
    equalityPoint = equalityQr.householderQ() * coordinates;
    checkDenseQpOriginalWorkingSet(original, equalityPoint,
        std::vector<bool>(original.bounds.size() - original.equalities, false));
    equalitiesReady = true;
  }

  bool prepareConstrainedDual() {
    if (constrainedDualReady) return dualR.cols() != 0;
    prepareEqualities();
    const Eigen::Index n = data.linear.size(), finite = objective.rows();
    Matrix augmented(finite + data.equalities, n + 1);
    augmented.topLeftCorner(finite, n) = objective;
    augmented.col(n).head(finite) = target(Vector::Zero(n));
    for (size_t row = 0; row < data.equalities; ++row) {
      augmented.row(finite + row).head(n) = data.normals.row(row) / equalityScales(row);
      augmented(finite + row, n) = data.bounds(row) / equalityScales(row);
    }
    Vector sigmas = Vector::Ones(augmented.rows());
    sigmas.tail(data.equalities).setZero();
    const auto model = noiseModel::internal::constrainedQR(
        augmented, sigmas, Vector::Constant(augmented.rows(), 1000));
    ++statistics.numericalFactorizations;
    ++statistics.objectiveFactorizations;
    constrainedDualReady = true;
    if (model->dim() != size_t(n)) return false;
    dualR = augmented.topLeftCorner(n, n).triangularView<Eigen::Upper>();
    dualVariances = model->sigmas().array().square().matrix();
    // A hard-pivot rank disagreement is numerical ambiguity, not curvature.
    if ((dualVariances.array() == 0).count() != equalityQr.rank()) {
      dualR.resize(0, 0);
      return false;
    }
    dualPoint = dualR.triangularView<Eigen::Upper>().solve(augmented.col(n).head(n));
    checkDenseQpOriginalWorkingSet(original, dualPoint,
        std::vector<bool>(original.bounds.size() - original.equalities, false));
    return true;
  }

  Vector constrainedResponse(const Vector& forcing) const {
    Vector forward = dualR.transpose().triangularView<Eigen::Lower>().solve(forcing);
    forward.array() *= dualVariances.array();
    return dualR.triangularView<Eigen::Upper>().solve(forward);
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
    if (data.nativeHessian.size()) {
      Matrix hessian = data.nativeHessian;
      hessian.diagonal().array() += data.regularization;
      Eigen::LLT<Matrix> cholesky(hessian);
      ++statistics.numericalFactorizations;
      if (cholesky.info() == Eigen::Success) {
        objectiveR = cholesky.matrixU();
        objectivePermutation.setIdentity(hessian.rows());
        nativeCholesky = invertible = objectiveReady = true;
        ++statistics.objectiveFactorizations;
        ++statistics.choleskyFactorizations;
        transformed.resize(data.bounds.size());
        scales.resize(data.bounds.size());
        unconstrainedTarget = objectiveR * nativeMinimum(Vector::Zero(hessian.rows()),
                                                         Vector::Zero(hessian.rows()));
        return;
      }
      // A failed native factorization retains the supplied square-root root.
    }
    objectiveQr.compute(objective);
    ++statistics.numericalFactorizations;
    ++statistics.objectiveFactorizations;
    invertible = objectiveQr.rank() == objective.cols();
    objectiveReady = true;
    objectivePermutation = objectiveQr.colsPermutation();
    transformed.resize(data.bounds.size());
    scales.resize(data.bounds.size());
    if (!invertible) return;
    const Eigen::Index n = objective.cols();
    objectiveR = objectiveQr.matrixR().topRows(n)
                     .triangularView<Eigen::Upper>();
    unconstrainedTarget = (objectiveQr.householderQ().adjoint() *
                           target(Vector::Zero(n))).head(n);
    unconstrainedTarget += objectiveR.transpose().triangularView<Eigen::Lower>()
        .solve(Vector(objectivePermutation.transpose() * data.linear));
    // Preserve weak RHS information before working constraints are imposed.
    const Vector value = objectivePermutation *
        objectiveR.triangularView<Eigen::Upper>().solve(unconstrainedTarget);
    const Vector refined = refineQrMinimum(objective, target(Vector::Zero(n)),
                                           data.linear, objectiveQr, value);
    unconstrainedTarget = objectiveR *
        (objectivePermutation.transpose() * refined);
  }

  // Apply cached curvature without materializing an inverse. Native Hessians
  // use the identity permutation; QR keeps its rank-revealing column order.
  Vector inverseResponse(const Vector& forcing) const {
    const Vector forward = objectiveR.transpose().triangularView<Eigen::Lower>()
        .solve(Vector(objectivePermutation.transpose() * forcing));
    return objectivePermutation * objectiveR.triangularView<Eigen::Upper>().solve(forward);
  }

  // Certify/refine against the original native Hessian rather than reconstruct
  // normal equations from a square root. Only the requested proximal term is
  // added, and center changes reuse both triangular factors.
  Vector nativeMinimum(const Vector& center, const Vector& forcing) const {
    const Vector linear = data.linear + forcing + data.regularization * center;
    Vector value = inverseResponse(linear);
    for (size_t refinement = 0; refinement < 2; ++refinement) {
      const Vector residual = linear - data.nativeHessian * value -
                              data.regularization * value;
      value += inverseResponse(residual);
    }
    return value;
  }

  // In whitened coordinates curvature is the identity. No inverse matrix is
  // formed, and each original normal needs only one cached triangular solve.
  const Vector& normal(size_t row) {
    if (transformed[row].size() == 0) {
      Vector value = objectiveR.transpose().triangularView<Eigen::Lower>().solve(
          Vector(objectivePermutation.transpose() *
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
          Vector(objectivePermutation.transpose() *
                 (data.regularization * center)));
    Vector rhs(basis.size());
    for (size_t i = 0; i < basis.size(); ++i)
      rhs(i) = data.bounds(basis[i]) / scales[basis[i]] - normal(basis[i]).dot(target);
    Vector multipliers = solveWorking(rhs);
    Vector value = target + workingNormals * multipliers;
    Vector result;
    for (size_t attempt = 0; attempt < 3; ++attempt) {
      result = objectivePermutation *
          objectiveR.triangularView<Eigen::Upper>().solve(value);
      if (attempt == 2 || basis.empty()) break;
      for (size_t i = 0; i < basis.size(); ++i)
        rhs(i) = (data.bounds(basis[i]) - data.normals.row(basis[i]).dot(result)) /
                 scales[basis[i]];
      // Exact residuals need no correction. An absolute cutoff would hide
      // weak independent boundaries whose small residual still matters.
      if (rhs.isZero(0)) break;
      const Vector correction = solveWorking(rhs);
      const Vector updated = value + workingNormals * correction;
      // Avoid another back-substitution if rounding leaves the point unchanged.
      if ((updated.array() == value.array()).all()) break;
      multipliers += correction;
      value = updated;
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
      try {
        check(value, rows);
        return value;
      }
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
    if (!projection && basis.empty()) return false;
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
        (result.tail(original.bounds.size() - original.equalities).array() > tolerance).any())
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

DenseQpSolver::DenseQpSolver(const DenseQpData& data, bool projectEqualities)
    : impl_(std::make_unique<Impl>(data, projectEqualities)) {}
DenseQpSolver::~DenseQpSolver() = default;
const DenseQpSolver::Statistics& DenseQpSolver::statistics() const {
  return impl_->statistics;
}
Vector DenseQpSolver::solve(const std::vector<bool>& active, const Vector& center) {
  // Numerical state belongs to this call, never the owning factor graph.
  return impl_->solveOriginal(active, center);
}

const DenseQpEqualityProjection* DenseQpSolver::equalityProjection() const {
  return impl_->projection ? &*impl_->projection : nullptr;
}

bool DenseQpSolver::canRecoverMultipliers() const {
  return impl_->canRecoverMultipliers();
}
std::optional<Vector> DenseQpSolver::multipliers(const Vector& value,
                                               const Vector& gradient,
                                               double tolerance) const {
  return impl_->multipliers(value, gradient, tolerance);
}

size_t DenseQpSolver::reducedDimension() const {
  if (impl_->data.equalities != 0) {
    impl_->prepareEqualities();
    return impl_->data.linear.size() - impl_->equalityQr.rank();
  }
  return impl_->data.linear.size();
}

bool DenseQpSolver::prepareDual() {
  (void)reducedDimension();
  if (impl_->data.equalities) return impl_->prepareConstrainedDual();
  if (impl_->data.linear.size() == 0) return true;
  if (!impl_->objectiveReady) impl_->prepareObjective();
  return impl_->invertible;
}

Vector DenseQpSolver::dualResponse(const Vector& forcing) const {
  const auto& impl = *impl_;
  if (impl.data.equalities) return impl.constrainedResponse(forcing);
  const auto* projection = impl.projection ? &*impl.projection : nullptr;
  const Vector reduced = projection
      ? Vector(projection->nullspace.transpose() * forcing) : forcing;
  if (reduced.size() == 0) return Vector::Zero(forcing.size());
  const Vector response = impl.inverseResponse(reduced);
  return projection ? Vector(projection->nullspace * response) : response;
}

Vector DenseQpSolver::solveDual(const Vector& center, const Vector& forcing) {
  auto& impl = *impl_;
  if (impl.nativeCholesky) {
    ++impl.statistics.rhsOnlySolves;
    return impl.nativeMinimum(center, forcing);
  }
  if (impl.data.equalities) {
    Vector value = impl.dualPoint;
    const Vector linear = impl.data.linear + forcing;
    for (size_t refinement = 0; refinement < 3; ++refinement) {
      const Vector gradient = impl.data.jacobian.transpose() *
          (impl.data.rhs - impl.data.jacobian * value) + linear +
          impl.data.regularization * (center - value);
      value += impl.constrainedResponse(gradient);
    }
    checkDenseQpOriginalWorkingSet(impl.original, value,
        std::vector<bool>(impl.original.bounds.size() - impl.original.equalities, false));
    ++impl.statistics.rhsOnlySolves;
    return value;
  }
  const auto* projection = impl.projection ? &*impl.projection : nullptr;
  if (impl.data.linear.size() == 0) return projection->point;
  const Vector reducedCenter = projection
      ? Vector(projection->nullspace.transpose() * (center - projection->point))
      : center;
  const Vector reducedForcing = projection
      ? Vector(projection->nullspace.transpose() * forcing) : forcing;
  const Vector rhs = impl.target(reducedCenter);
  Vector value = impl.objectiveQr.solve(rhs);
  const Vector linear = impl.data.linear + reducedForcing;
  value = refineQrMinimum(impl.objective, rhs, linear, impl.objectiveQr, value);
  ++impl.statistics.rhsOnlySolves;
  return projection ? Vector(projection->point + projection->nullspace * value)
                    : value;
}

Vector DenseQpSolver::equalityMultipliers(const Vector& gradient) const {
  if (impl_->data.equalities) {
    const auto& qr = impl_->equalityQr;
    const Eigen::Index rank = qr.rank();
    const Vector rotated = qr.householderQ().adjoint() * gradient;
    Vector ordered = Vector::Zero(impl_->data.equalities);
    ordered.head(rank) = qr.matrixR().topLeftCorner(rank, rank)
        .triangularView<Eigen::Upper>().solve(rotated.head(rank));
    return Vector(qr.colsPermutation() * ordered).cwiseQuotient(impl_->equalityScales);
  }
  return impl_->projection ? impl_->projection->multipliers(impl_->original, gradient)
                           : Vector();
}

}  // namespace internal
}  // namespace gtsam
