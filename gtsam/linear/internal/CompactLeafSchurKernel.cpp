/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CompactLeafSchurKernel.cpp
 * @brief Internal allocation-free kernels for compact leaf elimination.
 * @author Frank Dellaert
 */

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/linear/internal/CompactLeafSchurKernel.h>

#include <cassert>
#include <cmath>
#include <numeric>

namespace gtsam {
namespace internal {
namespace {

constexpr double kMinimumNormalizedPivotSquared = 1.0 / (4096.0 * 4096.0);

bool factorUpper(Matrix* matrix, DenseIndex dimension) {
  for (DenseIndex column = 0; column < dimension; ++column) {
    const double originalDiagonal = (*matrix)(column, column);
    if (!std::isfinite(originalDiagonal) || originalDiagonal <= 0.0) {
      return false;
    }

    double pivotSquared = originalDiagonal;
    for (DenseIndex k = 0; k < column; ++k) {
      pivotSquared -= (*matrix)(k, column) * (*matrix)(k, column);
    }
    if (!std::isfinite(pivotSquared) || pivotSquared <= 0.0 ||
        pivotSquared < kMinimumNormalizedPivotSquared * originalDiagonal) {
      return false;
    }

    const double pivot = std::sqrt(pivotSquared);
    (*matrix)(column, column) = pivot;
    for (DenseIndex targetColumn = column + 1; targetColumn < matrix->cols();
         ++targetColumn) {
      double value = (*matrix)(column, targetColumn);
      for (DenseIndex k = 0; k < column; ++k) {
        value -= (*matrix)(k, column) * (*matrix)(k, targetColumn);
      }
      (*matrix)(column, targetColumn) = value / pivot;
    }
  }
  matrix->topLeftCorner(dimension, dimension)
      .template triangularView<Eigen::StrictlyLower>()
      .setZero();
  return true;
}

}  // namespace

bool CompactLeafSchurKernel::factorFrontalRows(VerticalBlockMatrix* frontalRows,
                                               DenseIndex frontalDimension) {
  assert(frontalRows);
  Matrix& matrix = frontalRows->matrix();
  assert(frontalDimension >= 0 && frontalDimension <= matrix.rows());
  assert(matrix.rows() == frontalDimension);
  assert(matrix.cols() >= frontalDimension);
  return factorUpper(&matrix, frontalDimension);
}

std::vector<DenseIndex> CompactLeafSchurKernel::expandScalarOffsets(
    const std::vector<size_t>& blockDimensions,
    const std::vector<DenseIndex>& blockScalarOffsets) {
  assert(blockDimensions.size() == blockScalarOffsets.size());
  const size_t scalarCount = std::accumulate(
      blockDimensions.begin(), blockDimensions.end(), static_cast<size_t>(0));
  std::vector<DenseIndex> result;
  result.reserve(scalarCount);
  for (size_t block = 0; block < blockDimensions.size(); ++block) {
    for (size_t offset = 0; offset < blockDimensions[block]; ++offset) {
      result.push_back(blockScalarOffsets[block] +
                       static_cast<DenseIndex>(offset));
    }
  }
  return result;
}

void CompactLeafSchurKernel::subtractMappedOuterProduct(
    const VerticalBlockMatrix& frontalRows, DenseIndex frontalDimension,
    const std::vector<DenseIndex>& targetScalarOffsets,
    SymmetricBlockMatrix* target) {
  assert(target);
  const Matrix& source = frontalRows.matrix();
  const DenseIndex retainedColumns = source.cols() - frontalDimension;
  assert(retainedColumns ==
         static_cast<DenseIndex>(targetScalarOffsets.size()));

  for (DenseIndex column = 0; column < retainedColumns; ++column) {
    const DenseIndex targetColumn = targetScalarOffsets[column];
    assert(targetColumn >= 0 && targetColumn < target->matrix_.cols());
    for (DenseIndex row = 0; row <= column; ++row) {
      const DenseIndex targetRow = targetScalarOffsets[row];
      assert(targetRow >= 0 && targetRow < target->matrix_.rows());
      const double value =
          source.col(frontalDimension + row)
              .head(frontalDimension)
              .dot(
                  source.col(frontalDimension + column).head(frontalDimension));
      if (targetRow <= targetColumn) {
        target->matrix_(targetRow, targetColumn) -= value;
      } else {
        target->matrix_(targetColumn, targetRow) -= value;
      }
    }
  }
}

}  // namespace internal
}  // namespace gtsam
