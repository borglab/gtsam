/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file DenseQpEqualityProjection.cpp
 * One-time fixed-coordinate substitution and orthogonal equality elimination
 * for the production dense QP workspace. This code changes coordinates only;
 * the active-set policy and original-input certificates remain with its caller.
 */
#include <gtsam/constrained/internal/DenseQpEqualityProjection.h>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam::internal {
namespace {

// Recognize exact single-variable equations without a coefficient cutoff.
Eigen::Index singletonColumn(const Matrix& normals, Eigen::Index row) {
  Eigen::Index single = -1;
  for (Eigen::Index col = 0; col < normals.cols(); ++col) {
    if (normals(row, col) == 0) continue;
    if (single >= 0) return -1;
    single = col;
  }
  return single;
}

// Fixed coordinates are represented exactly, including homogeneous bounds.
std::vector<Eigen::Index> substituteFixedCoordinates(
    const DenseQpData& source, Vector* point,
    std::vector<Eigen::Index>* fixedRows) {
  const Eigen::Index n = source.linear.size();
  *point = Vector::Zero(n);
  fixedRows->assign(n, -1);
  std::vector<bool> fixed(n, false);
  for (size_t row = 0; row < source.equalities; ++row) {
    const Eigen::Index col = singletonColumn(source.normals, row);
    if (col < 0) continue;
    fixed[col] = true;
    (*fixedRows)[col] = row;
    (*point)(col) = source.bounds(row) / source.normals(row, col);
  }
  std::vector<Eigen::Index> free;
  for (Eigen::Index col = 0; col < n; ++col)
    if (!fixed[col]) free.push_back(col);
  return free;
}

// Rank decisions apply to normalized affine rows after exact substitution.
std::pair<Matrix, Vector> remainingEqualities(
    const DenseQpData& source, const Vector& point,
    const std::vector<Eigen::Index>& free, Vector* scales,
    std::vector<Eigen::Index>* originalRows) {
  Matrix normals(source.equalities, free.size());
  for (size_t col = 0; col < free.size(); ++col)
    normals.col(col) = source.normals.col(free[col]).head(source.equalities);
  Vector bounds = source.bounds.head(source.equalities) -
                  source.normals.topRows(source.equalities) * point;
  scales->resize(normals.rows());
  for (Eigen::Index row = 0; row < normals.rows(); ++row) {
    const double norm = normals.row(row).stableNorm();
    (*scales)(row) = norm > 0 ? norm : 1;
    if (norm > 0) {
      normals.row(row) /= norm;
      bounds(row) /= norm;
      originalRows->push_back(row);
    }
  }
  // Fixed-coordinate substitution creates literal zero rows. Compact only
  // those rows; every original boundary is checked after finding the point.
  Matrix compact(originalRows->size(), free.size());
  Vector compactBounds(originalRows->size());
  for (size_t i = 0; i < originalRows->size(); ++i) {
    compact.row(i) = normals.row((*originalRows)[i]);
    compactBounds(i) = bounds((*originalRows)[i]);
  }
  return {std::move(compact), std::move(compactBounds)};
}

// Lift the orthogonal free-coordinate basis with exact zeros at fixed keys.
void factorRemainingEqualities(const DenseQpData& source,
                              const std::vector<Eigen::Index>& free,
                              DenseQpEqualityProjection* projection) {
  if (free.empty()) {
    projection->nullspace.resize(source.linear.size(), 0);
    return;
  }
  std::vector<Eigen::Index> originalRows;
  const auto system = remainingEqualities(source, projection->point, free,
                                          &projection->equalityScales, &originalRows);
  // With only fixed-coordinate rows, the remaining coordinates are free.
  if (originalRows.empty()) {
    projection->equalityQ.resize(free.size(), 0);
    projection->equalityR.resize(0, 0);
    projection->nullspace = Matrix::Zero(source.linear.size(), free.size());
    for (size_t i = 0; i < free.size(); ++i) projection->nullspace(free[i], i) = 1;
    return;
  }
  Eigen::ColPivHouseholderQR<Matrix> qr(system.first.transpose());
  ++projection->numericalFactorizations;
  const Eigen::Index rank = qr.rank(), dimension = free.size();
  const Matrix orthogonal = qr.householderQ() * Matrix::Identity(dimension, dimension);
  projection->equalityQ = orthogonal.leftCols(rank);
  projection->equalityR = qr.matrixR().topLeftCorner(rank, rank)
                             .triangularView<Eigen::Upper>();
  for (Eigen::Index i = 0; i < rank; ++i)
    projection->equalityBasis.push_back(originalRows[qr.colsPermutation().indices()(i)]);
  const Vector ordered = qr.colsPermutation().transpose() * system.second;
  const Vector coordinates = qr.matrixR().topLeftCorner(rank, rank).transpose()
      .triangularView<Eigen::Lower>().solve(ordered.head(rank));
  const Vector freePoint = orthogonal.leftCols(rank) * coordinates;
  projection->nullspace = Matrix::Zero(source.linear.size(), dimension - rank);
  for (size_t row = 0; row < free.size(); ++row) {
    projection->point(free[row]) = freePoint(row);
    projection->nullspace.row(free[row]) = orthogonal.row(row).tail(dimension - rank);
  }
}

// A roundoff-sized projection can be either a redundant row or a genuine weak
// constraint. Keep the original dense system in that ambiguous case rather
// than normalize numerical noise or truncate a meaningful coefficient.
bool hasAmbiguousConstraintProjection(const DenseQpData& source,
                                     const Matrix& normals) {
  const double allowance = 64 * std::numeric_limits<double>::epsilon();
  for (Eigen::Index row = 0; row < normals.rows(); ++row) {
    const double projected = normals.row(row).stableNorm();
    const double original = source.normals.row(source.equalities + row).stableNorm();
    if (projected > 0 && projected <= allowance * original) return true;
  }
  return false;
}

// Constant objective residuals remain in original cost evaluation, but should
// not enter QR transformations of much weaker variable-dependent residuals.
void projectObjective(const DenseQpData& source,
                      DenseQpEqualityProjection* projection) {
  const Matrix jacobian = source.jacobian * projection->nullspace;
  const Vector rhs = source.rhs - source.jacobian * projection->point;
  std::vector<Eigen::Index> varying;
  for (Eigen::Index row = 0; row < jacobian.rows(); ++row)
    if (!jacobian.row(row).isZero(0)) varying.push_back(row);
  auto& data = projection->reduced;
  data.jacobian.resize(varying.size(), jacobian.cols());
  data.rhs.resize(varying.size());
  for (size_t row = 0; row < varying.size(); ++row) {
    data.jacobian.row(row) = jacobian.row(varying[row]);
    data.rhs(row) = rhs(varying[row]);
  }
  data.linear = projection->nullspace.transpose() * source.linear;
  data.regularization = source.regularization;
}

}  // namespace

Vector DenseQpEqualityProjection::multipliers(
    const DenseQpData& source, const Vector& gradient) const {
  Vector freeGradient(freeColumns.size());
  for (size_t i = 0; i < freeColumns.size(); ++i)
    freeGradient(i) = gradient(freeColumns[i]);
  const Vector basis = equalityR.triangularView<Eigen::Upper>().solve(
      Vector(equalityQ.transpose() * freeGradient));
  Vector result = Vector::Zero(source.equalities);
  for (size_t i = 0; i < equalityBasis.size(); ++i) {
    const auto row = equalityBasis[i];
    result(row) = basis(i) / equalityScales(row);
  }
  // Singleton rows absorb forcing at substituted coordinates, after the
  // remaining equations have accounted for their coupled contributions.
  const Vector residual = gradient -
      source.normals.topRows(source.equalities).transpose() * result;
  for (size_t col = 0; col < fixedRows.size(); ++col) {
    const auto row = fixedRows[col];
    if (row >= 0) result(row) += residual(col) / source.normals(row, col);
  }
  return result;
}

void checkDenseQpOriginalWorkingSet(const DenseQpData& source,
                                  const Vector& value,
                                  const std::vector<bool>& active) {
  if (active.size() + source.equalities != size_t(source.bounds.size()))
    throw std::invalid_argument("Dense QP: invalid working-set mask.");
  if (!value.allFinite())
    throw std::runtime_error("Dense QP: non-finite working-face solution.");
  for (size_t row = 0; row < size_t(source.bounds.size()); ++row) {
    if (row >= source.equalities && !active[row - source.equalities]) continue;
    const double scale = 1 + std::abs(source.bounds(row)) +
        source.normals.row(row).cwiseAbs().dot(value.cwiseAbs());
    const double allowance = 256 * std::numeric_limits<double>::epsilon() * scale;
    if (std::abs(source.normals.row(row).dot(value) - source.bounds(row)) >
        std::max(1e-10, allowance))
      throw std::runtime_error("Dense QP: inconsistent working constraints.");
  }
}

std::optional<DenseQpEqualityProjection> projectDenseQpEqualities(
    const DenseQpData& source) {
  if (source.equalities == 0) return std::nullopt;
  DenseQpEqualityProjection projection;
  projection.freeColumns = substituteFixedCoordinates(
      source, &projection.point, &projection.fixedRows);
  factorRemainingEqualities(source, projection.freeColumns, &projection);
  checkDenseQpOriginalWorkingSet(source, projection.point,
      std::vector<bool>(source.bounds.size() - source.equalities, false));
  const Eigen::Index count = source.bounds.size() - source.equalities;
  projection.reduced.normals = source.normals.bottomRows(count) * projection.nullspace;
  if (hasAmbiguousConstraintProjection(source, projection.reduced.normals))
    return std::nullopt;
  projectObjective(source, &projection);
  projection.reduced.bounds = source.bounds.tail(count) -
                             source.normals.bottomRows(count) * projection.point;
  return projection;
}

}  // namespace gtsam::internal
