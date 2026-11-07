/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file ConstrainedQR.cpp
 * Adapt Eigen QR to exact constraints and fixed-order, rank-aware elimination.
 */

#include <gtsam/linear/internal/ConstrainedQR.h>

#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace gtsam::noiseModel::internal {
namespace {

// GTSAM selects exact/finite pivots; Eigen performs all orthogonal transforms.
class ConstrainedQRWorkspace {
  Matrix& output_;
  const size_t variables_, maximumRank_;
  const double rankTolerance_;
  Matrix finite_, hard_, conditionals_;
  Vector reference_, precisions_, scratch_;
  Eigen::RowVectorXd row_;
  std::vector<size_t> pivots_;
  size_t finiteStart_ = 0, hardCount_ = 0;
  bool tryFullQR_ = true;

 public:
  explicit ConstrainedQRWorkspace(Matrix& augmented, const Vector& sigmas)
      : output_(augmented),
        variables_(augmented.cols() - 1),
        maximumRank_(std::min<size_t>(augmented.rows(), variables_)),
        rankTolerance_(64.0 * std::numeric_limits<double>::epsilon() *
                       std::max<size_t>(augmented.rows(), variables_)),
        conditionals_(maximumRank_, variables_ + 1),
        reference_(Vector::Zero(variables_)),
        precisions_(maximumRank_),
        scratch_(variables_ + 1),
        row_(variables_ + 1) {
    initializeRows(sigmas);
    pivots_.reserve(maximumRank_);
  }

  SharedDiagonal compute(const Vector& mu) {
    for (size_t column = 0;
         column < variables_ && pivots_.size() < maximumRank_; ++column) {
      if (const auto hardRow = selectHardRow(column))
        eliminateHard(column, *hardRow);
      else if (eliminateRemaining(column))
        break;
      else
        eliminateFinite(column);
    }
    // The output includes zero padding for skipped columns and redundant rows.
    output_.setZero();
    bool mixed = false;
    for (size_t i = 0; i < pivots_.size(); ++i) {
      const size_t column = pivots_[i];
      mixed |= std::isinf(precisions_(i));
      output_.row(i).segment(column, variables_ + 1 - column) =
          conditionals_.row(i).segment(column, variables_ + 1 - column);
    }
    precisions_.conservativeResize(pivots_.size());
    return mixed ? Constrained::MixedPrecisions(mu, precisions_)
                 : Diagonal::Precisions(precisions_);
  }

 private:
  // Whiten finite observations; exact equations retain their original scale.
  void initializeRows(const Vector& sigmas) {
    std::vector<size_t> finiteRows, hardRows;
    for (Eigen::Index i = 0; i < sigmas.size(); ++i) {
      const double inverse = 1.0 / sigmas(i), weight = inverse * inverse;
      if (std::isinf(inverse))
        hardRows.push_back(i);
      else if (std::isfinite(weight) && weight > 0.0)
        finiteRows.push_back(i);
    }
    finite_.resize(finiteRows.size(), variables_ + 1);
    hardCount_ = hardRows.size();
    hard_.resize(hardCount_, variables_ + 1);
    for (size_t i = 0; i < finiteRows.size(); ++i) {
      const double inverse = 1.0 / sigmas(finiteRows[i]);
      finite_.row(i) = std::abs(inverse) * output_.row(finiteRows[i]);
      reference_.array() += inverse * inverse *
                            output_.row(finiteRows[i])
                                .head(variables_)
                                .transpose()
                                .array()
                                .square();
    }
    for (size_t i = 0; i < hardCount_; ++i)
      hard_.row(i) = output_.row(hardRows[i]);
  }

  // Keep the existing hard coefficient cutoff and weighted rank criterion.
  std::optional<size_t> selectHardRow(size_t column) const {
    if (!hardCount_) return {};
    Eigen::Index selected;
    if (hard_.col(column).head(hardCount_).cwiseAbs().maxCoeff(&selected) >
        1e-9)
      return selected;
    return {};
  }

  bool accepted(double precision, size_t column) const {
    return std::isfinite(precision) && precision > 0.0 &&
           std::sqrt(precision) >
               rankTolerance_ * std::sqrt(reference_(column));
  }

  void append(size_t column, double precision) {
    conditionals_.row(pivots_.size()).segment(column, variables_ + 1 - column) =
        row_.segment(column, variables_ + 1 - column);
    precisions_(pivots_.size()) = precision;
    pivots_.push_back(column);
  }

  // Substitute an accepted equation, without changing the variable ordering.
  void substitute(Matrix& matrix, size_t start, size_t count, size_t column,
                  double inversePivot = 1.0) const {
    matrix.block(start, column + 1, count, variables_ - column).noalias() -=
        matrix.col(column).segment(start, count) *
        (inversePivot * row_.tail(variables_ - column));
  }

  void eliminateHard(size_t column, size_t selected) {
    row_ = hard_.row(selected);
    append(column, std::numeric_limits<double>::infinity());
    --hardCount_;
    if (selected != hardCount_) hard_.row(selected) = hard_.row(hardCount_);
    // Exact substitution can transfer finite information to later columns.
    const size_t remaining = variables_ - column - 1;
    reference_.tail(remaining) =
        (reference_.tail(remaining).array().sqrt() +
         (row_.segment(column + 1, remaining).transpose().array() /
          row_(column))
                 .abs() *
             std::sqrt(reference_(column)))
            .square();
    substitute(finite_, finiteStart_, finite_.rows() - finiteStart_, column,
               1.0 / row_(column));
    substitute(hard_, 0, hardCount_, column, 1.0 / row_(column));
  }

  void eliminateFinite(size_t column) {
    const size_t count = finite_.rows() - finiteStart_;
    if (!count ||
        !accepted(finite_.col(column).tail(count).squaredNorm(), column))
      return;
    // Put the largest observation first so a zero leading entry cannot mix an
    // unrelated RHS into the pivot. Eigen constructs and applies the reflector.
    Eigen::Index pivot;
    finite_.col(column).tail(count).cwiseAbs().maxCoeff(&pivot);
    finite_.row(finiteStart_).swap(finite_.row(finiteStart_ + pivot));
    // Pack only contributing observations; Eigen then transforms a contiguous
    // block without visiting structurally zero rows in every trailing column.
    size_t support = 1;
    for (size_t i = 1; i < count; ++i) {
      if (finite_(finiteStart_ + i, column) == 0.0) continue;
      if (i != support)
        finite_.row(finiteStart_ + i)
            .tail(variables_ + 1 - column)
            .swap(finite_.row(finiteStart_ + support)
                      .tail(variables_ + 1 - column));
      ++support;
    }
    auto pivotColumn = finite_.col(column).segment(finiteStart_, support);
    double tau, diagonal;
    pivotColumn.makeHouseholderInPlace(tau, diagonal);
    finite_.block(finiteStart_, column + 1, support, variables_ - column)
        .applyHouseholderOnTheLeft(pivotColumn.tail(support - 1), tau,
                                   scratch_.data());
    row_(column) = 1.0;
    row_.tail(variables_ - column) =
        finite_.row(finiteStart_).tail(variables_ - column) / diagonal;
    append(column, diagonal * diagonal);
    substitute(hard_, 0, hardCount_, column);
    ++finiteStart_;
  }

  bool eliminateRemaining(size_t column) {
    const size_t width = variables_ - column,
                 count = finite_.rows() - finiteStart_;
    if (!tryFullQR_ || width < 48 || count < width ||
        !hard_.block(0, column, hardCount_, width).isZero(0.0))
      return false;
    // The weak and nearly dependent RHS regressions require row-pivoted scalar
    // elimination. Otherwise delegate the entire remaining finite system.
    // A rejected remainder is tried only once, avoiding repeated failed QR
    // work.
    tryFullQR_ = false;
    const auto scales = reference_.tail(width);
    const double epsilon = std::numeric_limits<double>::epsilon();
    if (scales.minCoeff() < epsilon * scales.maxCoeff()) return false;
    Eigen::HouseholderQR<Matrix> qr(
        finite_.block(finiteStart_, column, count, width));
    for (size_t k = 0; k < width; ++k) {
      const double precision = qr.matrixQR()(k, k) * qr.matrixQR()(k, k);
      if (!accepted(precision, column + k) ||
          precision < epsilon * reference_(column + k))
        return false;
    }
    const Vector rhs =
        qr.householderQ().adjoint() * finite_.col(variables_).tail(count);
    for (size_t k = 0; k < width; ++k) {
      const double diagonal = qr.matrixQR()(k, k);
      row_.segment(column + k, width - k) =
          qr.matrixQR().row(k).segment(k, width - k) / diagonal;
      row_(variables_) = rhs(k) / diagonal;
      append(column + k, diagonal * diagonal);
    }
    return true;
  }
};

}  // namespace

SharedDiagonal constrainedQR(Matrix& augmented, const Vector& sigmas,
                             const Vector& mu) {
  return ConstrainedQRWorkspace(augmented, sigmas).compute(mu);
}

}  // namespace gtsam::noiseModel::internal
