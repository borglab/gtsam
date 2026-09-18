/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceSparseSpdSystem.h
 * @brief   Persistent device storage for an int32 upper-CSR SPD system
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSystem.h>

#include <cstddef>
#include <vector>

namespace gtsam::cuda {

/**
 * Persistent device storage for an int32 upper-CSR SPD system.
 *
 * The sparsity pattern and the numbers stored in it have deliberately different
 * lifetimes, which is what the accessors below encode. The pattern is immutable
 * between uploadPattern() calls: it can only be established by uploadPattern(),
 * which validates it and sizes the value and RHS storage from it, so
 * rowPointers() and colIndices() are const-only. The numerics are rewritten
 * every iteration, so values() and rhs() also have mutable overloads. Keeping
 * the pattern behind that one entry point is why this is a class rather than a
 * struct of public arrays: a caller that could resize colIndices() without
 * touching rowPointers(), or store a descending row pointer, would first find
 * out when a kernel read out of bounds on the device. Frontends that just want
 * the flat arrays should use view().
 *
 * Mutating operations enqueue work on the supplied CUDA stream. The caller must
 * keep host inputs alive until the queued upload completes and must order later
 * access on the same stream or with explicit CUDA events. Views and array
 * references borrow this object's storage and are invalidated by pattern
 * upload, move, or destruction.
 */
class GTSAM_EXPORT DeviceSparseSpdSystem {
 public:
  /**
   * Validates and uploads an upper-CSR sparsity pattern, then allocates values
   * and RHS storage. Existing storage and any borrowed views are invalidated.
   *
   * The pattern must satisfy rows >= 0, rowPointers.size() == rows + 1,
   * rowPointers.front() == 0, non-decreasing row pointers within
   * [0, colIndices.size()], rowPointers.back() == colIndices.size(), and every
   * column index in [0, rows). The nonzero count must be representable as int,
   * because cuDSS and the kernels index with int32. A violation throws
   * std::invalid_argument and leaves existing storage untouched.
   *
   * transferProfile performs synchronous profiled uploads. Alternatively,
   * copyBeginEvent and copyEndEvent may both be supplied to bracket the queued
   * copies; the two profiling mechanisms are mutually exclusive.
   */
  void uploadPattern(int rows, const std::vector<int>& rowPointers,
                     const std::vector<int>& colIndices,
                     cudaStream_t stream = nullptr,
                     DeviceTransferSummary* transferProfile = nullptr,
                     cudaEvent_t copyBeginEvent = nullptr,
                     cudaEvent_t copyEndEvent = nullptr);

  /// Returns the matrix dimension.
  int rows() const { return rows_; }
  /// Returns the stored upper-triangle nonzero count.
  int nonzeros() const { return static_cast<int>(values_.size()); }

  /// Asynchronously zeros matrix values and the right-hand side.
  void zero(cudaStream_t stream = nullptr) {
    values_.zero(stream);
    rhs_.zero(stream);
  }

  /// Adds lambda to every stored diagonal entry.
  void addDiagonalDamping(double lambda, cudaStream_t stream = nullptr);
  /// Adds lambda times diagonal to every stored diagonal entry.
  void addDiagonalDamping(double lambda,
                          const DeviceArray<double>& diagonal,
                          cudaStream_t stream = nullptr);
  /// Saves the current matrix diagonal for repeated damping attempts.
  void captureUndampedDiagonal(cudaStream_t stream = nullptr);
  /// Restores the saved diagonal and adds scalar damping.
  void restoreAndAddDiagonal(double lambda, cudaStream_t stream = nullptr);
  /// Restores the saved diagonal and adds element-wise damping.
  void restoreAndAddDiagonal(double lambda,
                             const DeviceArray<double>& diagonal,
                             cudaStream_t stream = nullptr);

  /**
   * Returns a borrowed upper-CSR solver view of the current storage.
   * The RHS is mutable because solver sessions may overwrite it.
   */
  SparseSpdSystemView view() {
    return {rows_, nonzeros(), rowPointers_.data(), colIndices_.data(),
            values_.data(), rhs_.data(), SparseTriangle::Upper};
  }

  /// Borrows device row pointers.
  const DeviceArray<int>& rowPointers() const { return rowPointers_; }
  /// Borrows device column indices.
  const DeviceArray<int>& colIndices() const { return colIndices_; }
  /// Borrows mutable device matrix values.
  DeviceArray<double>& values() { return values_; }
  /// Borrows device matrix values.
  const DeviceArray<double>& values() const { return values_; }
  /// Borrows the mutable device right-hand side.
  DeviceArray<double>& rhs() { return rhs_; }
  /// Borrows the device right-hand side.
  const DeviceArray<double>& rhs() const { return rhs_; }
  /// Borrows the saved undamped diagonal, if one has been captured.
  const DeviceArray<double>& undampedDiagonal() const {
    return undampedDiagonal_;
  }

 private:
  int rows_ = 0;
  DeviceArray<int> rowPointers_;
  DeviceArray<int> colIndices_;
  DeviceArray<double> values_;
  DeviceArray<double> rhs_;
  DeviceArray<double> undampedDiagonal_;
};

}  // namespace gtsam::cuda
