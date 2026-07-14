#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam::cuda {

class DeviceSparseNormalEquations {
 public:
  void uploadPattern(int rows, const std::vector<int>& rowPointers,
                     const std::vector<int>& colIndices,
                     cudaStream_t stream = nullptr,
                     CudaDeviceTransferSummary* transferProfile = nullptr,
                     cudaEvent_t copyBeginEvent = nullptr,
                     cudaEvent_t copyEndEvent = nullptr) {
    if (rows < 0) {
      throw std::invalid_argument("DeviceSparseNormalEquations rows < 0");
    }
    if (rowPointers.size() != static_cast<size_t>(rows) + 1) {
      throw std::invalid_argument(
          "DeviceSparseNormalEquations bad rowPointers");
    }
    if (colIndices.size() >
        static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument("DeviceSparseNormalEquations too many nnz");
    }
    if (!rowPointers.empty() && rowPointers.front() != 0) {
      throw std::invalid_argument(
          "DeviceSparseNormalEquations bad rowPointers");
    }
    for (size_t i = 1; i < rowPointers.size(); ++i) {
      if (rowPointers[i] < rowPointers[i - 1]) {
        throw std::invalid_argument(
            "DeviceSparseNormalEquations bad rowPointers");
      }
      if (rowPointers[i] < 0 ||
          rowPointers[i] > static_cast<int>(colIndices.size())) {
        throw std::invalid_argument(
            "DeviceSparseNormalEquations bad rowPointers");
      }
    }
    if (!rowPointers.empty() &&
        rowPointers.back() != static_cast<int>(colIndices.size())) {
      throw std::invalid_argument("DeviceSparseNormalEquations bad nnz");
    }
    for (int col : colIndices) {
      if (col < 0 || col >= rows) {
        throw std::invalid_argument(
            "DeviceSparseNormalEquations bad column index");
      }
    }
    if ((copyBeginEvent == nullptr) != (copyEndEvent == nullptr)) {
      throw std::invalid_argument(
          "DeviceSparseNormalEquations requires both copy events");
    }
    if (transferProfile && copyBeginEvent) {
      throw std::invalid_argument(
          "DeviceSparseNormalEquations cannot combine transfer profiles");
    }

    CudaDeviceArray<int> newRowPointers;
    CudaDeviceArray<int> newColIndices;
    CudaDeviceArray<double> newValues;
    CudaDeviceArray<double> newRhs;

    try {
      if (copyBeginEvent) {
        // Allocate every persistent array before recording the event so this
        // interval measures only the two pattern copies.
        newRowPointers.resize(rowPointers.size());
        newColIndices.resize(colIndices.size());
        newValues.resize(colIndices.size());
        newRhs.resize(rows);
        GTSAM_CUDA_CHECK(cudaEventRecord(copyBeginEvent, stream));
        if (!rowPointers.empty()) {
          GTSAM_CUDA_CHECK(cudaMemcpyAsync(newRowPointers.data(),
                                           rowPointers.data(),
                                           sizeof(int) * rowPointers.size(),
                                           cudaMemcpyHostToDevice, stream));
        }
        if (!colIndices.empty()) {
          GTSAM_CUDA_CHECK(cudaMemcpyAsync(
              newColIndices.data(), colIndices.data(),
              sizeof(int) * colIndices.size(), cudaMemcpyHostToDevice, stream));
        }
        GTSAM_CUDA_CHECK(cudaEventRecord(copyEndEvent, stream));
      } else if (transferProfile) {
        transferProfile->add(
            newRowPointers.uploadProfiled(rowPointers, stream));
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
      // Keep the temporary allocations and pageable upload sources alive until
      // any successfully queued copy has finished. Preserve the original
      // exception even if synchronization also reports an error.
      cudaStreamSynchronize(stream);
      throw;
    }

    rowPointers_ = std::move(newRowPointers);
    colIndices_ = std::move(newColIndices);
    values_ = std::move(newValues);
    rhs_ = std::move(newRhs);
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
                          const CudaDeviceArray<double>& diagonal,
                          cudaStream_t stream = nullptr);

  const CudaDeviceArray<int>& rowPointers() const { return rowPointers_; }
  const CudaDeviceArray<int>& colIndices() const { return colIndices_; }
  CudaDeviceArray<double>& values() { return values_; }
  const CudaDeviceArray<double>& values() const { return values_; }
  CudaDeviceArray<double>& rhs() { return rhs_; }
  const CudaDeviceArray<double>& rhs() const { return rhs_; }

 private:
  int rows_ = 0;
  CudaDeviceArray<int> rowPointers_;
  CudaDeviceArray<int> colIndices_;
  CudaDeviceArray<double> values_;
  CudaDeviceArray<double> rhs_;
};

}  // namespace gtsam::cuda
