/* GTSAM Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#include <gtsam/base/CholeskyCheck.h>
#include <gtsam/base/cuda/Errors.h>
#include <gtsam/linear/cuda/internal/CholeskyCheck.h>

namespace gtsam::cuda::internal {
namespace {
constexpr int kBlockSize = 256;

__global__ void saveDiagonalKernel(const double* matrix, int dimension,
                                   int stride, double* diagonal) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < dimension) diagonal[i] = matrix[static_cast<size_t>(i) * stride + i];
}

__global__ void checkPivotsKernel(const double* factor, int dimension,
                                  int stride, const double* reference,
                                  int* info) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < dimension &&
      !gtsam::internal::CholeskyCheck::accept(
          factor[static_cast<size_t>(i) * stride + i], reference[i]))
    atomicCAS(info, 0, i + 1);
}
}  // namespace

void saveCholeskyDiagonal(const double* matrix, int dimension, int stride,
                          double* diagonal, cudaStream_t stream) {
  if (dimension <= 0) return;
  saveDiagonalKernel<<<(dimension + kBlockSize - 1) / kBlockSize, kBlockSize, 0,
                       stream>>>(matrix, dimension, stride, diagonal);
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

void checkCholeskyPivots(const double* factor, int dimension, int stride,
                         const double* reference, int* info,
                         cudaStream_t stream) {
  if (dimension <= 0) return;
  checkPivotsKernel<<<(dimension + kBlockSize - 1) / kBlockSize, kBlockSize, 0,
                      stream>>>(factor, dimension, stride, reference, info);
  GTSAM_CUDA_CHECK(cudaGetLastError());
}
}  // namespace gtsam::cuda::internal
