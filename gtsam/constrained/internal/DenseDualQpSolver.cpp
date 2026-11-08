/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <gtsam/constrained/internal/DenseDualQpSolver.h>
#include <gtsam/constrained/internal/ActiveSetSolverInternal.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace gtsam::internal {
namespace {
using Clock = std::chrono::steady_clock;

// Keep timing ownership local, including early returns and fallback exits.
struct Elapsed {
  double& seconds;
  Clock::time_point began = Clock::now();
  bool running = true;
  void stop() {
    if (running) seconds += std::chrono::duration<double>(Clock::now() - began).count();
    running = false;
  }
  ~Elapsed() { stop(); }
};
}  // namespace

struct DenseDualQpSolver::Impl {
  DenseQpSolver& dense;
  const DenseQpSolver::Whitening& coordinates;
  DualQpSolver::Statistics& statistics;
  std::vector<size_t> rows;
  Matrix lower, gram;
  Vector diagonal, equalityMultipliers;
  bool nonsingular = true, initialized = false;

  Impl(DenseQpSolver& workspace, DualQpSolver::Statistics& counters)
      : dense(workspace), coordinates(*workspace.prepareWhitening()), statistics(counters),
        equalityMultipliers(Vector::Zero(coordinates.equalities)) {
    Elapsed elapsed{statistics.equalitySeconds};
    if (!coordinates.equalities) return;
    // Reuse independent original equality pivots. Bordering verifies their
    // transformed rank, so a second equality QR is unnecessary.
    for (size_t row : coordinates.equalityBasis) append(row);
    if (!nonsingular)
      throw WhiteningFailure("Dense Dual: transformed equalities are numerically dependent.");
  }

  double threshold() const {
    return 128 * std::numeric_limits<double>::epsilon() *
        std::max<Eigen::Index>(1, gram.rows()) *
        (gram.size() ? gram.diagonal().cwiseAbs().maxCoeff() : 0);
  }

  // Border with an immutable equality or working inequality. Keep a singular
  // border for the null-direction step rather than silently discarding a row.
  void append(size_t row) {
    const size_t count = rows.size();
    const auto column = coordinates.normals.row(row);
    Vector cross(count);
    for (size_t i = 0; i < count; ++i)
      cross(i) = coordinates.normals.row(rows[i]).dot(column);
    const double pivot = column.squaredNorm();
    gram.conservativeResize(count + 1, count + 1);
    gram.topRightCorner(count, 1) = cross;
    gram.bottomLeftCorner(1, count) = cross.transpose();
    gram(count, count) = pivot;
    rows.push_back(row);
    ++statistics.rowAdditions;
    if (!nonsingular) return;
    Vector border = lower.triangularView<Eigen::UnitLower>().solve(cross);
    if (count) border.array() /= diagonal.array();
    const double remainder = pivot - border.dot(diagonal.cwiseProduct(border));
    lower.conservativeResize(count + 1, count + 1);
    lower.topRightCorner(count, 1).setZero();
    lower.bottomLeftCorner(1, count) = border.transpose();
    lower(count, count) = 1;
    diagonal.conservativeResize(count + 1);
    diagonal(count) = remainder;
    nonsingular = remainder > threshold();
  }

  void rebuild() {
    ++statistics.workingRebuilds;
    lower = Matrix::Identity(rows.size(), rows.size());
    diagonal.resize(rows.size());
    nonsingular = true;
    for (size_t i = 0; i < rows.size(); ++i) {
      Vector border = lower.topLeftCorner(i, i).triangularView<Eigen::UnitLower>()
          .solve(Vector(gram.col(i).head(i)));
      if (i) border.array() /= diagonal.head(i).array();
      diagonal(i) = gram(i, i) - border.dot(diagonal.head(i).cwiseProduct(border));
      lower.row(i).head(i) = border.transpose();
      if (diagonal(i) <= threshold()) { nonsingular = false; break; }
    }
  }

  // Deleting a working row is a rank-one update of the trailing LDL factors.
  void remove(size_t position) {
    const Eigen::Index count = rows.size(), remaining = count - position - 1;
    Matrix compact(count - 1, count - 1);
    for (Eigen::Index i = 0; i < count - 1; ++i)
      for (Eigen::Index j = 0; j < count - 1; ++j)
        compact(i, j) = gram(i + (i >= Eigen::Index(position)),
                              j + (j >= Eigen::Index(position)));
    gram = std::move(compact);
    rows.erase(rows.begin() + position);
    ++statistics.rowRemovals;
    if (!nonsingular) { rebuild(); return; }
    Vector update = lower.col(position).tail(remaining);
    double weight = diagonal(position);
    Matrix next = Matrix::Identity(count - 1, count - 1);
    next.topLeftCorner(position, position) = lower.topLeftCorner(position, position);
    next.bottomLeftCorner(remaining, position) = lower.bottomLeftCorner(remaining, position);
    next.bottomRightCorner(remaining, remaining) = lower.bottomRightCorner(remaining, remaining);
    Vector pivots(count - 1);
    pivots.head(position) = diagonal.head(position);
    pivots.tail(remaining) = diagonal.tail(remaining);
    for (Eigen::Index i = 0; i < remaining; ++i) {
      const Eigen::Index index = position + i, trailing = remaining - i - 1;
      const double previous = pivots(index), component = update(i);
      pivots(index) += weight * component * component;
      const double coefficient = weight * component / pivots(index);
      weight *= previous / pivots(index);
      update.tail(trailing) -= component * next.col(index).tail(trailing);
      next.col(index).tail(trailing) += coefficient * update.tail(trailing);
    }
    lower = std::move(next);
    diagonal = std::move(pivots);
  }

  // Return either a stationary point or a strict null-space descent direction.
  std::pair<Vector, bool> workingSolution(const Vector& linear) const {
    if (rows.empty()) return {Vector(), false};
    if (nonsingular) {
      Vector value = lower.triangularView<Eigen::UnitLower>().solve(Vector(-linear));
      value.array() /= diagonal.array();
      return {lower.transpose().triangularView<Eigen::UnitUpper>().solve(value), false};
    }
    Eigen::SelfAdjointEigenSolver<Matrix> eigen(gram);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < -threshold())
      throw WhiteningFailure("Dense Dual: numerically indefinite transformed working system.");
    const Vector projected = eigen.eigenvectors().transpose() * linear;
    Vector solution = Vector::Zero(linear.size()), direction = solution;
    for (Eigen::Index i = 0; i < linear.size(); ++i) {
      if (eigen.eigenvalues()(i) > threshold())
        solution(i) = -projected(i) / eigen.eigenvalues()(i);
      else direction(i) = -projected(i);
    }
    const double allowance = 256 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, linear.norm());
    if (direction.norm() > allowance)
      return {eigen.eigenvectors() * direction, true};
    return {eigen.eigenvectors() * solution, false};
  }

  Vector transformedPoint(const Vector& target, const Vector& multipliers) {
    ++statistics.transformedPointUpdates;
    Vector result = target;
    for (size_t i = 0; i < rows.size(); ++i)
      result.noalias() -= multipliers(i) * coordinates.normals.row(rows[i]).transpose();
    return result;
  }

  Vector allMultipliers(const Vector& working) const {
    Vector result = Vector::Zero(coordinates.bounds.size());
    for (size_t i = 0; i < rows.size(); ++i)
      result(rows[i]) = working(i) / coordinates.scales(rows[i]);
    return result;
  }

  // Verify a singular descent direction against original rows, never just the
  // transformed Gram matrix. Permanent equality multipliers have either sign.
  void checkRay(const Vector& direction) const {
    const auto& original = dense.originalData();
    Vector multipliers = allMultipliers(direction / direction.lpNorm<Eigen::Infinity>());
    Vector normal = Vector::Zero(original.linear.size());
    double separation = 0, scale = 0, normalScale = 0;
    for (Eigen::Index row = 0; row < multipliers.size(); ++row) {
      const bool equality = size_t(row) < coordinates.equalities;
      if (!equality && multipliers(row) < 0)
        throw WhiteningFailure("Dense Dual: invalid transformed infeasibility ray.");
      const size_t source = equality ? size_t(row)
          : original.equalities + row - coordinates.equalities;
      const Vector term = multipliers(row) * original.normals.row(source).transpose();
      normal += term;
      normalScale += term.norm();
      const double contribution = multipliers(row) * original.bounds(source);
      separation += contribution;
      scale += std::abs(contribution);
    }
    if (!coordinates.equalities && original.equalities) {
      const Vector equalities = dense.equalityMultipliers(normal);
      normal -= original.normals.topRows(original.equalities).transpose() * equalities;
      for (size_t row = 0; row < original.equalities; ++row) {
        const double contribution = equalities(row) * original.bounds(row);
        separation -= contribution;
        scale += std::abs(contribution);
      }
    }
    const double roundoff = 1024 * std::numeric_limits<double>::epsilon();
    if (!normal.allFinite() || !std::isfinite(separation) ||
        normal.norm() > roundoff * std::max(1.0, normalScale) ||
        separation >= -roundoff * std::max(1.0, scale))
      throw WhiteningFailure("Dense Dual: ambiguous transformed infeasibility ray.");
    throw std::runtime_error("Dual QP: infeasible inequality constraints (verified dual ray).");
  }

  // Recover only at a transformed optimum. Cancellation in the original
  // objective or boundaries triggers residual correction with cached factors.
  // An uncertain correction exits to the established original-coordinate path.
  Vector certify(Vector value, const Vector& center, Vector* working,
                 double feasibilityTolerance, double multiplierTolerance) {
    Elapsed elapsed{statistics.recoverySeconds};
    const auto& original = dense.originalData();
    for (size_t attempt = 0; attempt < 3; ++attempt) {
      Vector candidate = dense.recoverWhitening(value);
      const Vector residual = original.normals * candidate - original.bounds;
      if (!residual.allFinite()) break;
      double maximum = 0;
      for (Eigen::Index row = 0; row < residual.size(); ++row)
        maximum = std::max(maximum, size_t(row) < original.equalities
            ? std::abs(residual(row)) : residual(row));
      double complementarity = 0;
      for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i] < coordinates.equalities) continue;
        const size_t source = original.equalities + rows[i] - coordinates.equalities;
        maximum = std::max(maximum, std::abs(residual(source)));
        complementarity += std::abs((*working)(i) / coordinates.scales(rows[i]) *
                                    residual(source));
      }
      const Vector stationarity = dense.whiteningCorrection(candidate, center,
                                                            allMultipliers(*working));
      // A correction is measured in objective square-root coordinates. Always
      // keep the final original-QP stationarity/gap certificate authoritative.
      const double allowance = 64 * std::numeric_limits<double>::epsilon() *
                               (1 + value.norm());
      if (candidate.allFinite() && maximum <= feasibilityTolerance &&
          complementarity <= multiplierTolerance &&
          stationarity.norm() <= std::max(allowance, multiplierTolerance * 0.125))
        return candidate;
      if (!nonsingular || attempt == 2 || !stationarity.allFinite()) break;
      ++statistics.conditionalRefinements;
      value += stationarity;
      candidate = dense.recoverWhitening(value);
      const Vector errors = dense.whiteningResidual(candidate);
      Vector rhs(rows.size());
      for (size_t i = 0; i < rows.size(); ++i)
        rhs(i) = errors(rows[i]) / coordinates.scales(rows[i]);
      const Vector correction = workingSolution(-rhs).first;
      *working += correction;
      for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i] >= coordinates.equalities && (*working)(i) < 0)
          throw WhiteningFailure("Dense Dual: refinement changes multiplier feasibility.");
        value.noalias() -= correction(i) * coordinates.normals.row(rows[i]).transpose();
      }
    }
    throw WhiteningFailure("Dense Dual: transformed candidate needs original-coordinate refinement.");
  }

  Vector solve(const Vector& center, Vector* multipliers, std::vector<bool>* active,
               size_t* iterations, size_t maxIterations,
               double feasibilityTolerance, double multiplierTolerance) {
    Elapsed iterationTime{statistics.iterationSeconds};
    if (!initialized) {
      for (size_t row = 0; row < active->size(); ++row)
        if ((*active)[row]) append(coordinates.equalities + row);
      initialized = true;
    }
    const Vector target = dense.whiteningTarget(center);
    const Vector linearAll = coordinates.bounds.cwiseQuotient(coordinates.scales) -
                             coordinates.normals * target;
    while (*iterations < maxIterations) {
      ++*iterations;
      ++statistics.whitenedWorkingSolves;
      Vector linear(rows.size()), current(rows.size());
      for (size_t i = 0; i < rows.size(); ++i) {
        const size_t row = rows[i];
        linear(i) = linearAll(row);
        current(i) = row < coordinates.equalities ? equalityMultipliers(row)
            : (*multipliers)(row - coordinates.equalities) * coordinates.scales(row);
      }
      auto [targetMultipliers, ray] = workingSolution(linear);
      const Vector direction = ray ? targetMultipliers : Vector(targetMultipliers - current);
      double step = ray ? std::numeric_limits<double>::infinity() : 1;
      size_t leaving = rows.size();
      for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i] < coordinates.equalities || direction(i) >= 0 ||
            (!ray && targetMultipliers(i) >= 0)) continue;
        const double candidate = -current(i) / direction(i);
        if (candidate < step || (candidate == step &&
            (leaving == rows.size() || rows[i] < rows[leaving]))) {
          step = candidate;
          leaving = i;
        }
      }
      if (ray && leaving == rows.size()) checkRay(direction);
      Vector next = current + step * direction;
      for (size_t i = 0; i < rows.size(); ++i) {
        const size_t row = rows[i];
        if (row < coordinates.equalities) equalityMultipliers(row) = next(i);
        else (*multipliers)(row - coordinates.equalities) =
            std::max(0.0, next(i) / coordinates.scales(row));
      }
      if (leaving != rows.size()) {
        const size_t row = rows[leaving] - coordinates.equalities;
        (*multipliers)(row) = 0;
        (*active)[row] = false;
        remove(leaving);
        continue;
      }
      const Vector value = transformedPoint(target, next);
      if (!value.allFinite()) throw WhiteningFailure("Dense Dual: non-finite transformed point.");
      // Immutable equalities are already enforced by the working system.
      // DAQP's violation scan only visits inequality rows.
      const Eigen::Index inequalities = active->size();
      const Vector violation = (coordinates.normals.bottomRows(inequalities) * value)
          .cwiseProduct(coordinates.scales.tail(inequalities)) -
          coordinates.bounds.tail(inequalities);
      if (!violation.allFinite())
        throw WhiteningFailure("Dense Dual: non-finite transformed residual.");
      size_t entering = active->size();
      double largest = feasibilityTolerance;
      for (size_t row = 0; row < active->size(); ++row) {
        if (!(*active)[row] && violation(row) > largest) {
          largest = violation(row);
          entering = row;
        }
      }
      if (entering != active->size()) {
        (*active)[entering] = true;
        append(coordinates.equalities + entering);
        continue;
      }
      iterationTime.stop();
      const Vector result = certify(value, center, &next, feasibilityTolerance, multiplierTolerance);
      for (size_t i = 0; i < rows.size(); ++i) {
        const size_t row = rows[i];
        if (row < coordinates.equalities) equalityMultipliers(row) = next(i);
        else (*multipliers)(row - coordinates.equalities) = next(i) / coordinates.scales(row);
      }
      return result;
    }
    iterationTime.stop();
    throw IterationLimitError("ActiveSetSolver: dual QP failed to converge.");
  }
};

DenseDualQpSolver::DenseDualQpSolver(DenseQpSolver& dense,
                                   DualQpSolver::Statistics& statistics)
    : impl_(std::make_unique<Impl>(dense, statistics)) {}
DenseDualQpSolver::~DenseDualQpSolver() = default;
Vector DenseDualQpSolver::solve(const Vector& center, Vector* multipliers,
    std::vector<bool>* active, size_t* iterations, size_t maxIterations,
    double feasibilityTolerance, double multiplierTolerance) {
  return impl_->solve(center, multipliers, active, iterations, maxIterations,
                      feasibilityTolerance, multiplierTolerance);
}

}  // namespace gtsam::internal
