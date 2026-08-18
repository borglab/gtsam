#include <gtsam/base/cuda/Errors.h>
#include <gtsam/linear/cuda/SparseSpdOperator.h>

#include <stdexcept>

namespace gtsam::cuda {
namespace {
constexpr int kBlockSize = 256;

__global__ void symmetricUpperCsrMatvecKernel(
    int rows, const int* rowPointers, const int* columns,
    const double* values, const double* input, double* output) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;
  double rowValue = 0.0;
  for (int entry = rowPointers[row]; entry < rowPointers[row + 1]; ++entry) {
    const int column = columns[entry];
    const double value = values[entry];
    rowValue += value * input[column];
    if (column != row) atomicAdd(&output[column], value * input[row]);
  }
  atomicAdd(&output[row], rowValue);
}

__global__ void buildInverseDiagonalKernel(
    int rows, const int* rowPointers, const int* columns,
    const double* values, double* inverseDiagonal) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows) return;
  double diagonal = 0.0;
  for (int entry = rowPointers[row]; entry < rowPointers[row + 1]; ++entry) {
    if (columns[entry] == row) {
      diagonal = values[entry];
      break;
    }
  }
  inverseDiagonal[row] = diagonal > 0.0 ? 1.0 / diagonal : 1.0;
}

__global__ void applyInverseDiagonalKernel(int count, const double* inverse,
                                            const double* input,
                                            double* output) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < count) output[i] = inverse[i] * input[i];
}

int grid(int count) { return (count + kBlockSize - 1) / kBlockSize; }
}  // namespace

void SparseSpdOperator::apply(const double* input, double* output,
                                  cudaStream_t stream) const {
  if (!input || !output) throw std::invalid_argument("sparse SPD matvec null storage");
  const int rows = system_->rows();
  GTSAM_CUDA_CHECK(cudaMemsetAsync(output, 0, sizeof(double) * rows, stream));
  if (rows > 0) {
    symmetricUpperCsrMatvecKernel<<<grid(rows), kBlockSize, 0, stream>>>(
        rows, system_->rowPointers().data(), system_->colIndices().data(),
        system_->values().data(), input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

void SparseSpdJacobiPreconditioner::build(
    const DeviceSparseSpdSystem& system, cudaStream_t stream) {
  dimension_ = system.rows();
  inverseDiagonal_.resize(static_cast<size_t>(dimension_));
  if (dimension_ > 0) {
    buildInverseDiagonalKernel<<<grid(dimension_), kBlockSize, 0, stream>>>(
        dimension_, system.rowPointers().data(), system.colIndices().data(),
        system.values().data(), inverseDiagonal_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

void SparseSpdJacobiPreconditioner::apply(
    const double* input, double* output, cudaStream_t stream) const {
  if (inverseDiagonal_.size() != static_cast<size_t>(dimension_))
    throw std::logic_error("sparse SPD Jacobi preconditioner is not built");
  if (dimension_ > 0) {
    applyInverseDiagonalKernel<<<grid(dimension_), kBlockSize, 0, stream>>>(
        dimension_, inverseDiagonal_.data(), input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

}  // namespace gtsam::cuda
