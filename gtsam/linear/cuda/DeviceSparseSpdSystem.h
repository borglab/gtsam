#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSystem.h>

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam::cuda {

/** Persistent device storage for an int32 upper-CSR SPD system. */
class GTSAM_EXPORT DeviceSparseSpdSystem {
 public:
  void uploadPattern(int rows, const std::vector<int>& rowPointers,
                     const std::vector<int>& colIndices,
                     cudaStream_t stream = nullptr,
                     DeviceTransferSummary* transferProfile = nullptr,
                     cudaEvent_t copyBeginEvent = nullptr,
                     cudaEvent_t copyEndEvent = nullptr) {
    if (rows < 0) {
      throw std::invalid_argument("DeviceSparseSpdSystem rows < 0");
    }
    if (rowPointers.size() != static_cast<size_t>(rows) + 1) {
      throw std::invalid_argument("DeviceSparseSpdSystem bad rowPointers");
    }
    if (colIndices.size() >
        static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument("DeviceSparseSpdSystem too many nnz");
    }
    if (!rowPointers.empty() && rowPointers.front() != 0) {
      throw std::invalid_argument("DeviceSparseSpdSystem bad rowPointers");
    }
    for (size_t i = 1; i < rowPointers.size(); ++i) {
      if (rowPointers[i] < rowPointers[i - 1] || rowPointers[i] < 0 ||
          rowPointers[i] > static_cast<int>(colIndices.size())) {
        throw std::invalid_argument("DeviceSparseSpdSystem bad rowPointers");
      }
    }
    if (!rowPointers.empty() &&
        rowPointers.back() != static_cast<int>(colIndices.size())) {
      throw std::invalid_argument("DeviceSparseSpdSystem bad nnz");
    }
    for (int col : colIndices) {
      if (col < 0 || col >= rows) {
        throw std::invalid_argument(
            "DeviceSparseSpdSystem bad column index");
      }
    }
    if ((copyBeginEvent == nullptr) != (copyEndEvent == nullptr)) {
      throw std::invalid_argument(
          "DeviceSparseSpdSystem requires both copy events");
    }
    if (transferProfile && copyBeginEvent) {
      throw std::invalid_argument(
          "DeviceSparseSpdSystem cannot combine transfer profiles");
    }

    DeviceArray<int> newRowPointers;
    DeviceArray<int> newColIndices;
    DeviceArray<double> newValues;
    DeviceArray<double> newRhs;

    try {
      if (copyBeginEvent) {
        newRowPointers.resize(rowPointers.size());
        newColIndices.resize(colIndices.size());
        newValues.resize(colIndices.size());
        newRhs.resize(rows);
        GTSAM_CUDA_CHECK(cudaEventRecord(copyBeginEvent, stream));
        if (!rowPointers.empty()) {
          GTSAM_CUDA_CHECK(cudaMemcpyAsync(
              newRowPointers.data(), rowPointers.data(),
              sizeof(int) * rowPointers.size(), cudaMemcpyHostToDevice,
              stream));
        }
        if (!colIndices.empty()) {
          GTSAM_CUDA_CHECK(cudaMemcpyAsync(
              newColIndices.data(), colIndices.data(),
              sizeof(int) * colIndices.size(), cudaMemcpyHostToDevice,
              stream));
        }
        GTSAM_CUDA_CHECK(cudaEventRecord(copyEndEvent, stream));
      } else if (transferProfile) {
        transferProfile->add(newRowPointers.uploadProfiled(rowPointers, stream));
        transferProfile->add(newColIndices.uploadProfiled(colIndices, stream));
      } else {
        newRowPointers.upload(rowPointers, stream);
        newColIndices.upload(colIndices, stream);
      }
      if (!copyBeginEvent) {
        newValues.resize(colIndices.size());
        newRhs.resize(rows);
      }
    } catch (...) {
      cudaStreamSynchronize(stream);
      throw;
    }

    rowPointers_ = std::move(newRowPointers);
    colIndices_ = std::move(newColIndices);
    values_ = std::move(newValues);
    rhs_ = std::move(newRhs);
    undampedDiagonal_.reset();
    rows_ = rows;
  }

  int rows() const { return rows_; }
  int nonzeros() const { return static_cast<int>(values_.size()); }

  void zero(cudaStream_t stream = nullptr) {
    values_.zero(stream);
    rhs_.zero(stream);
  }

  void addDiagonalDamping(double lambda, cudaStream_t stream = nullptr);
  void addDiagonalDamping(double lambda,
                          const DeviceArray<double>& diagonal,
                          cudaStream_t stream = nullptr);
  void captureUndampedDiagonal(cudaStream_t stream = nullptr);
  void restoreAndAddDiagonal(double lambda, cudaStream_t stream = nullptr);
  void restoreAndAddDiagonal(double lambda,
                             const DeviceArray<double>& diagonal,
                             cudaStream_t stream = nullptr);

  SparseSpdSystemView view() {
    return {rows_, nonzeros(), rowPointers_.data(), colIndices_.data(),
            values_.data(), rhs_.data(), SparseTriangle::Upper};
  }

  const DeviceArray<int>& rowPointers() const { return rowPointers_; }
  const DeviceArray<int>& colIndices() const { return colIndices_; }
  DeviceArray<double>& values() { return values_; }
  const DeviceArray<double>& values() const { return values_; }
  DeviceArray<double>& rhs() { return rhs_; }
  const DeviceArray<double>& rhs() const { return rhs_; }
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
