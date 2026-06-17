#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kDiagonalDampingBlockSize = 256;

__device__ int FindDiagonalEntry(const int* rowPointers, const int* colIndices,
                                 int row) {
  const int begin = rowPointers[row];
  const int end = rowPointers[row + 1];
  for (int entry = begin; entry < end; ++entry) {
    if (colIndices[entry] == row) {
      return entry;
    }
  }
  return -1;
}

__global__ void CountMissingDiagonalsKernel(int rows, const int* rowPointers,
                                            const int* colIndices,
                                            int* missingDiagonals) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;

  if (FindDiagonalEntry(rowPointers, colIndices, row) < 0) {
    atomicAdd(missingDiagonals, 1);
  }
}

__global__ void AddDiagonalDampingKernel(int rows, const int* rowPointers,
                                         const int* colIndices, double* values,
                                         double lambda) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;

  const int diagonalEntry = FindDiagonalEntry(rowPointers, colIndices, row);
  if (diagonalEntry >= 0) {
    values[diagonalEntry] += lambda;
  }
}

}  // namespace

void DeviceSparseNormalEquations::addDiagonalDamping(double lambda,
                                                     cudaStream_t stream) {
  if (rows_ == 0) {
    return;
  }
  if (rowPointers_.size() != static_cast<size_t>(rows_) + 1 ||
      colIndices_.size() != values_.size()) {
    throw std::invalid_argument(
        "DeviceSparseNormalEquations damping CSR size mismatch");
  }

  const size_t gridSizeSize =
      (static_cast<size_t>(rows_) + kDiagonalDampingBlockSize - 1) /
      kDiagonalDampingBlockSize;
  if (gridSizeSize > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(
        "DeviceSparseNormalEquations damping grid size exceeds CUDA launch "
        "limit");
  }

  CudaDeviceArray<int> missingDiagonals(1);
  missingDiagonals.zero(stream);

  CountMissingDiagonalsKernel<<<static_cast<int>(gridSizeSize),
                                kDiagonalDampingBlockSize, 0, stream>>>(
      rows_, rowPointers_.data(), colIndices_.data(),
      missingDiagonals.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());

  std::vector<int> hostMissingDiagonals;
  missingDiagonals.download(&hostMissingDiagonals, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  if (!hostMissingDiagonals.empty() && hostMissingDiagonals[0] != 0) {
    throw std::runtime_error(
        "DeviceSparseNormalEquations missing diagonal entries");
  }

  AddDiagonalDampingKernel<<<static_cast<int>(gridSizeSize),
                             kDiagonalDampingBlockSize, 0, stream>>>(
      rows_, rowPointers_.data(), colIndices_.data(), values_.data(), lambda);
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

}  // namespace gtsam::cuda
