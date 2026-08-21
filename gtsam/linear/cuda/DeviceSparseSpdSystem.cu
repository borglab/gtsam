/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceSparseSpdSystem.cu
 * @brief   Persistent device storage for an int32 upper-CSR SPD system
 * @author  Ruogu Li
 * @date    Aug 15, 2026
 */

#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kDiagonalBlockSize = 256;

__device__ int findDiagonalEntry(const int* rowPointers,
                                 const int* colIndices, int row) {
  for (int entry = rowPointers[row]; entry < rowPointers[row + 1]; ++entry) {
    if (colIndices[entry] == row) return entry;
  }
  return -1;
}

__global__ void countMissingDiagonalsKernel(int rows, const int* rowPointers,
                                            const int* colIndices,
                                            int* missingDiagonals) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row < rows && findDiagonalEntry(rowPointers, colIndices, row) < 0) {
    atomicAdd(missingDiagonals, 1);
  }
}

__global__ void addDiagonalKernel(int rows, const int* rowPointers,
                                  const int* colIndices, double* values,
                                  double lambda, const double* diagonal) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;
  const int entry = findDiagonalEntry(rowPointers, colIndices, row);
  if (entry >= 0) values[entry] += lambda * (diagonal ? diagonal[row] : 1.0);
}

__global__ void captureDiagonalKernel(int rows, const int* rowPointers,
                                      const int* colIndices,
                                      const double* values, double* diagonal) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;
  const int entry = findDiagonalEntry(rowPointers, colIndices, row);
  if (entry >= 0) diagonal[row] = values[entry];
}

__global__ void restoreDiagonalKernel(int rows, const int* rowPointers,
                                      const int* colIndices, double* values,
                                      const double* undamped, double lambda,
                                      const double* damping) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;
  const int entry = findDiagonalEntry(rowPointers, colIndices, row);
  if (entry >= 0) {
    values[entry] = undamped[row] + lambda * (damping ? damping[row] : 1.0);
  }
}

int gridSize(int rows) {
  const size_t size =
      (static_cast<size_t>(rows) + kDiagonalBlockSize - 1) /
      kDiagonalBlockSize;
  if (size > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(
        "DeviceSparseSpdSystem diagonal grid exceeds CUDA launch limit");
  }
  return static_cast<int>(size);
}

void validateStorage(const DeviceSparseSpdSystem& system) {
  if (system.rowPointers().size() !=
          static_cast<size_t>(system.rows()) + 1 ||
      system.colIndices().size() != system.values().size()) {
    throw std::invalid_argument(
        "DeviceSparseSpdSystem diagonal CSR size mismatch");
  }
}

void validateDiagonals(const DeviceSparseSpdSystem& system,
                       cudaStream_t stream) {
  if (system.rows() == 0) return;
  DeviceArray<int> missing(1);
  missing.zero(stream);
  countMissingDiagonalsKernel<<<gridSize(system.rows()), kDiagonalBlockSize, 0,
                                stream>>>(
      system.rows(), system.rowPointers().data(), system.colIndices().data(),
      missing.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
  std::vector<int> hostMissing;
  missing.download(&hostMissing, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  if (hostMissing[0] != 0) {
    throw std::runtime_error(
        "DeviceSparseSpdSystem missing diagonal entries");
  }
}

}  // namespace

void DeviceSparseSpdSystem::uploadPattern(
    int rows, const std::vector<int>& rowPointers,
    const std::vector<int>& colIndices, cudaStream_t stream,
    DeviceTransferSummary* transferProfile, cudaEvent_t copyBeginEvent,
    cudaEvent_t copyEndEvent) {
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
      throw std::invalid_argument("DeviceSparseSpdSystem bad column index");
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

  // Build into fresh arrays and commit by move, so a failed allocation or copy
  // leaves the existing pattern and storage intact.
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
            sizeof(int) * rowPointers.size(), cudaMemcpyHostToDevice, stream));
      }
      if (!colIndices.empty()) {
        GTSAM_CUDA_CHECK(cudaMemcpyAsync(
            newColIndices.data(), colIndices.data(),
            sizeof(int) * colIndices.size(), cudaMemcpyHostToDevice, stream));
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
    // The queued copies may still reference the host vectors, which the caller
    // is free to destroy once this throws.
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

void DeviceSparseSpdSystem::addDiagonalDamping(double lambda,
                                               cudaStream_t stream) {
  if (rows_ == 0) return;
  validateStorage(*this);
  validateDiagonals(*this, stream);
  addDiagonalKernel<<<gridSize(rows_), kDiagonalBlockSize, 0, stream>>>(
      rows_, rowPointers_.data(), colIndices_.data(), values_.data(), lambda,
      nullptr);
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

void DeviceSparseSpdSystem::addDiagonalDamping(
    double lambda, const DeviceArray<double>& diagonal,
    cudaStream_t stream) {
  if (rows_ == 0) return;
  if (diagonal.size() != static_cast<size_t>(rows_)) {
    throw std::invalid_argument(
        "DeviceSparseSpdSystem damping diagonal size mismatch");
  }
  validateStorage(*this);
  validateDiagonals(*this, stream);
  addDiagonalKernel<<<gridSize(rows_), kDiagonalBlockSize, 0, stream>>>(
      rows_, rowPointers_.data(), colIndices_.data(), values_.data(), lambda,
      diagonal.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

void DeviceSparseSpdSystem::captureUndampedDiagonal(cudaStream_t stream) {
  if (rows_ == 0) {
    undampedDiagonal_.reset();
    return;
  }
  validateStorage(*this);
  validateDiagonals(*this, stream);
  undampedDiagonal_.resize(rows_);
  captureDiagonalKernel<<<gridSize(rows_), kDiagonalBlockSize, 0, stream>>>(
      rows_, rowPointers_.data(), colIndices_.data(), values_.data(),
      undampedDiagonal_.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

void DeviceSparseSpdSystem::restoreAndAddDiagonal(double lambda,
                                                  cudaStream_t stream) {
  if (rows_ == 0) return;
  if (undampedDiagonal_.size() != static_cast<size_t>(rows_)) {
    throw std::logic_error(
        "DeviceSparseSpdSystem undamped diagonal was not captured");
  }
  restoreDiagonalKernel<<<gridSize(rows_), kDiagonalBlockSize, 0, stream>>>(
      rows_, rowPointers_.data(), colIndices_.data(), values_.data(),
      undampedDiagonal_.data(), lambda, nullptr);
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

void DeviceSparseSpdSystem::restoreAndAddDiagonal(
    double lambda, const DeviceArray<double>& diagonal,
    cudaStream_t stream) {
  if (rows_ == 0) return;
  if (undampedDiagonal_.size() != static_cast<size_t>(rows_)) {
    throw std::logic_error(
        "DeviceSparseSpdSystem undamped diagonal was not captured");
  }
  if (diagonal.size() != static_cast<size_t>(rows_)) {
    throw std::invalid_argument(
        "DeviceSparseSpdSystem damping diagonal size mismatch");
  }
  restoreDiagonalKernel<<<gridSize(rows_), kDiagonalBlockSize, 0, stream>>>(
      rows_, rowPointers_.data(), colIndices_.data(), values_.data(),
      undampedDiagonal_.data(), lambda, diagonal.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

}  // namespace gtsam::cuda
