/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockTridiagonalCholeskySolver.cpp
 * @brief   Persistent cuSOLVER/cuBLAS batched block-Cholesky backend for B
 *          independent block-tridiagonal SPD systems of the same shape
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <gtsam/linear/cuda/internal/BatchedBlockTridiagonalCholeskySolver.h>

#include <gtsam/base/cuda/Errors.h>

#include <chrono>
#include <cublas_v2.h>
#include <cusolverDn.h>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {
namespace {

const char* cusolverStatusName(cusolverStatus_t status) {
  switch (status) {
    case CUSOLVER_STATUS_SUCCESS:
      return "CUSOLVER_STATUS_SUCCESS";
    case CUSOLVER_STATUS_NOT_INITIALIZED:
      return "CUSOLVER_STATUS_NOT_INITIALIZED";
    case CUSOLVER_STATUS_ALLOC_FAILED:
      return "CUSOLVER_STATUS_ALLOC_FAILED";
    case CUSOLVER_STATUS_INVALID_VALUE:
      return "CUSOLVER_STATUS_INVALID_VALUE";
    case CUSOLVER_STATUS_ARCH_MISMATCH:
      return "CUSOLVER_STATUS_ARCH_MISMATCH";
    case CUSOLVER_STATUS_EXECUTION_FAILED:
      return "CUSOLVER_STATUS_EXECUTION_FAILED";
    case CUSOLVER_STATUS_INTERNAL_ERROR:
      return "CUSOLVER_STATUS_INTERNAL_ERROR";
    default:
      return "CUSOLVER_STATUS_UNKNOWN";
  }
}

void checkCusolver(cusolverStatus_t status, const char* expression) {
  if (status == CUSOLVER_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuSOLVER call failed: " << expression << " returned "
     << cusolverStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

const char* cublasStatusName(cublasStatus_t status) {
  switch (status) {
    case CUBLAS_STATUS_SUCCESS:
      return "CUBLAS_STATUS_SUCCESS";
    case CUBLAS_STATUS_NOT_INITIALIZED:
      return "CUBLAS_STATUS_NOT_INITIALIZED";
    case CUBLAS_STATUS_ALLOC_FAILED:
      return "CUBLAS_STATUS_ALLOC_FAILED";
    case CUBLAS_STATUS_INVALID_VALUE:
      return "CUBLAS_STATUS_INVALID_VALUE";
    case CUBLAS_STATUS_ARCH_MISMATCH:
      return "CUBLAS_STATUS_ARCH_MISMATCH";
    case CUBLAS_STATUS_EXECUTION_FAILED:
      return "CUBLAS_STATUS_EXECUTION_FAILED";
    case CUBLAS_STATUS_INTERNAL_ERROR:
      return "CUBLAS_STATUS_INTERNAL_ERROR";
    default:
      return "CUBLAS_STATUS_UNKNOWN";
  }
}

void checkCublas(cublasStatus_t status, const char* expression) {
  if (status == CUBLAS_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuBLAS call failed: " << expression << " returned "
     << cublasStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

#define GTSAM_CUSOLVER_CHECK(expr) checkCusolver((expr), #expr)
#define GTSAM_CUBLAS_CHECK(expr) checkCublas((expr), #expr)

double elapsed(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       start)
      .count();
}

/// Fills a persistent device pointer array (one pointer per batch item,
/// base + i*strideElements), same rationale as BatchedDenseCholeskySolver's
/// identical helper: avoids a fresh cudaMalloc/cudaFree per chain step.
void uploadBatchPointers(double* base, int batchSize, size_t strideElements,
                        cudaStream_t stream, DeviceArray<double*>* cache) {
  std::vector<double*> hostPointers(static_cast<size_t>(batchSize));
  for (int i = 0; i < batchSize; ++i) {
    hostPointers[static_cast<size_t>(i)] =
        base + static_cast<size_t>(i) * strideElements;
  }
  cache->upload(hostPointers, stream);
}

}  // namespace

struct BatchedBlockTridiagonalCholeskySolver::Impl {
  cusolverDnHandle_t solverHandle = nullptr;
  cublasHandle_t blasHandle = nullptr;
  // Persistent pointer-array caches for TRSM/potrf (no strided-batched
  // variant exists for either); GEMM below uses cublasDgemmStridedBatched
  // directly, with no pointer array.
  DeviceArray<double*> ptrsA;
  DeviceArray<double*> ptrsB;
  DeviceArray<double*> ptrsC;
  DeviceArray<int> potrfInfo;
  double* diagonalBase = nullptr;
  double* offDiagonalBase = nullptr;
  int batchSize = 0;
  int numBlocks = 0;
  int blockDim = 0;
  bool factorized = false;
  // -1 means "whatever device is already current" (see
  // BatchedDenseSolverOptions); otherwise re-asserted at the top of every
  // public entry point below, not just here at construction -- cudaSetDevice
  // is thread-global, so a second same-thread instance constructed on a
  // different device would otherwise silently leave this one's later calls
  // running against the wrong device's memory.
  int deviceId = -1;

  explicit Impl(const BatchedDenseSolverOptions& options)
      : deviceId(options.deviceId) {
    if (deviceId >= 0) {
      GTSAM_CUDA_CHECK(cudaSetDevice(deviceId));
    }
    GTSAM_CUSOLVER_CHECK(cusolverDnCreate(&solverHandle));
    GTSAM_CUBLAS_CHECK(cublasCreate(&blasHandle));
  }
  ~Impl() {
    if (blasHandle) cublasDestroy(blasHandle);
    if (solverHandle) cusolverDnDestroy(solverHandle);
  }

  /// Device pointer to batch item i's block t (stride numBlocks*blockDim^2
  /// between consecutive items, matching the batch-item-major layout the
  /// header documents).
  double* diagonalBlockBase(int t) const {
    return diagonalBase + static_cast<size_t>(t) *
                              static_cast<size_t>(blockDim) * blockDim;
  }
  double* offDiagonalBlockBase(int t) const {
    return offDiagonalBase + static_cast<size_t>(t) *
                                  static_cast<size_t>(blockDim) * blockDim;
  }
  size_t diagonalBatchStride() const {
    return static_cast<size_t>(numBlocks) * blockDim * blockDim;
  }
  size_t offDiagonalBatchStride() const {
    return static_cast<size_t>(numBlocks - 1) * blockDim * blockDim;
  }
};

BatchedBlockTridiagonalCholeskySolver::BatchedBlockTridiagonalCholeskySolver(
    const BatchedDenseSolverOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
BatchedBlockTridiagonalCholeskySolver::
    ~BatchedBlockTridiagonalCholeskySolver() = default;
BatchedBlockTridiagonalCholeskySolver::BatchedBlockTridiagonalCholeskySolver(
    BatchedBlockTridiagonalCholeskySolver&&) noexcept = default;
BatchedBlockTridiagonalCholeskySolver&
BatchedBlockTridiagonalCholeskySolver::operator=(
    BatchedBlockTridiagonalCholeskySolver&&) noexcept = default;

void BatchedBlockTridiagonalCholeskySolver::factorize(
    int batchSize, int numBlocks, int blockDim,
    DeviceArray<double>* diagonalBlocks,
    DeviceArray<double>* offDiagonalBlocks, cudaStream_t stream,
    LinearSolveStats* stats) {
  if (batchSize <= 0 || numBlocks <= 1 || blockDim <= 0 || !diagonalBlocks ||
      !offDiagonalBlocks ||
      diagonalBlocks->size() != static_cast<size_t>(batchSize) *
                                     static_cast<size_t>(numBlocks) *
                                     static_cast<size_t>(blockDim) *
                                     static_cast<size_t>(blockDim) ||
      offDiagonalBlocks->size() != static_cast<size_t>(batchSize) *
                                        static_cast<size_t>(numBlocks - 1) *
                                        static_cast<size_t>(blockDim) *
                                        static_cast<size_t>(blockDim)) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalCholeskySolver::factorize requires "
        "numBlocks > 1 and diagonalBlocks/offDiagonalBlocks sized "
        "batchSize*numBlocks*blockDim*blockDim / "
        "batchSize*(numBlocks-1)*blockDim*blockDim");
  }
  if (impl_->deviceId >= 0) {
    GTSAM_CUDA_CHECK(cudaSetDevice(impl_->deviceId));
  }
  const auto start = std::chrono::steady_clock::now();
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(impl_->solverHandle, stream));
  GTSAM_CUBLAS_CHECK(cublasSetStream(impl_->blasHandle, stream));

  impl_->diagonalBase = diagonalBlocks->data();
  impl_->offDiagonalBase = offDiagonalBlocks->data();
  impl_->batchSize = batchSize;
  impl_->numBlocks = numBlocks;
  impl_->blockDim = blockDim;
  impl_->factorized = false;

  const size_t diagStride = impl_->diagonalBatchStride();
  const size_t offDiagStride = impl_->offDiagonalBatchStride();
  // One info slot per chain step, not shared: all potrfBatched calls queue
  // on the same stream with no sync between them. Downloading/checking all
  // of them once at the end (not per-step) avoids serializing the chain.
  impl_->potrfInfo.resize(static_cast<size_t>(batchSize) *
                         static_cast<size_t>(numBlocks));

  const double one = 1.0;
  const double minusOne = -1.0;

  // L_00 = chol(H_00).
  uploadBatchPointers(impl_->diagonalBlockBase(0), batchSize, diagStride,
                     stream, &impl_->ptrsC);
  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrfBatched(
      impl_->solverHandle, CUBLAS_FILL_MODE_LOWER, blockDim,
      impl_->ptrsC.data(), blockDim, impl_->potrfInfo.data(), batchSize));

  // Z_t = L_{t-1,t-1}^{-1} H_{t-1,t} (TRSM); L_tt = chol(H_tt - Z_t^T Z_t).
  // GEMM uses cublasDgemmStridedBatched (no pointer array); TRSM/potrf
  // have no strided-batched form in cuBLAS/cuSOLVER, so still need one.
  for (int t = 1; t < numBlocks; ++t) {
    // Z_t = L_{t-1,t-1}^{-1} H_{t-1,t}, overwriting H_{t-1,t} in place.
    uploadBatchPointers(impl_->diagonalBlockBase(t - 1), batchSize,
                       diagStride, stream, &impl_->ptrsA);
    uploadBatchPointers(impl_->offDiagonalBlockBase(t - 1), batchSize,
                       offDiagStride, stream, &impl_->ptrsB);
    GTSAM_CUBLAS_CHECK(cublasDtrsmBatched(
        impl_->blasHandle, CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
        CUBLAS_OP_N, CUBLAS_DIAG_NON_UNIT, blockDim, blockDim, &one,
        impl_->ptrsA.data(), blockDim, impl_->ptrsB.data(), blockDim,
        batchSize));

    // H_tt updated in place to H_tt - Z_t^T Z_t.
    GTSAM_CUBLAS_CHECK(cublasDgemmStridedBatched(
        impl_->blasHandle, CUBLAS_OP_T, CUBLAS_OP_N, blockDim, blockDim,
        blockDim, &minusOne, impl_->offDiagonalBlockBase(t - 1), blockDim,
        static_cast<long long>(offDiagStride),
        impl_->offDiagonalBlockBase(t - 1), blockDim,
        static_cast<long long>(offDiagStride), &one,
        impl_->diagonalBlockBase(t), blockDim,
        static_cast<long long>(diagStride), batchSize));

    // L_tt = chol(H_tt - Z_t^T Z_t). Each step's info lands in its own
    // batchSize-wide slot -- see the comment on potrfInfo's resize above.
    uploadBatchPointers(impl_->diagonalBlockBase(t), batchSize, diagStride,
                       stream, &impl_->ptrsC);
    GTSAM_CUSOLVER_CHECK(cusolverDnDpotrfBatched(
        impl_->solverHandle, CUBLAS_FILL_MODE_LOWER, blockDim,
        impl_->ptrsC.data(), blockDim,
        impl_->potrfInfo.data() + static_cast<size_t>(t) * batchSize,
        batchSize));
  }

  std::vector<int> hostInfo;
  impl_->potrfInfo.download(&hostInfo, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  for (int t = 0; t < numBlocks; ++t) {
    for (int i = 0; i < batchSize; ++i) {
      const int info =
          hostInfo[static_cast<size_t>(t) * batchSize + static_cast<size_t>(i)];
      if (info != 0) {
        std::ostringstream os;
        os << "BatchedBlockTridiagonalCholeskySolver::factorize: batch item "
           << i << " block " << t << " is not positive definite (info="
           << info << ")";
        throw std::runtime_error(os.str());
      }
    }
  }

  impl_->factorized = true;
  if (stats) {
    stats->backend = LinearSolverType::DenseCholesky;
    ++stats->factorizationCount;
    stats->factorizationSeconds += elapsed(start);
  }
}

void BatchedBlockTridiagonalCholeskySolver::solve(int k,
                                                  DeviceArray<double>* rhs,
                                                  cudaStream_t stream,
                                                  LinearSolveStats* stats) {
  if (!impl_->factorized) {
    throw std::logic_error(
        "BatchedBlockTridiagonalCholeskySolver::solve called before "
        "factorize");
  }
  const int batchSize = impl_->batchSize;
  const int numBlocks = impl_->numBlocks;
  const int blockDim = impl_->blockDim;
  if (k <= 0 || !rhs ||
      rhs->size() != static_cast<size_t>(batchSize) *
                         static_cast<size_t>(numBlocks) *
                         static_cast<size_t>(blockDim) *
                         static_cast<size_t>(k)) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalCholeskySolver::solve requires rhs sized "
        "batchSize*numBlocks*blockDim*k matching the factorized batch");
  }
  if (impl_->deviceId >= 0) {
    GTSAM_CUDA_CHECK(cudaSetDevice(impl_->deviceId));
  }
  const auto start = std::chrono::steady_clock::now();
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(impl_->solverHandle, stream));
  GTSAM_CUBLAS_CHECK(cublasSetStream(impl_->blasHandle, stream));

  const size_t diagStride = impl_->diagonalBatchStride();
  const size_t offDiagStride = impl_->offDiagonalBatchStride();
  const size_t rhsBlockStride =
      static_cast<size_t>(blockDim) * static_cast<size_t>(k);
  const size_t rhsBatchStride =
      static_cast<size_t>(numBlocks) * rhsBlockStride;
  const double one = 1.0;
  const double minusOne = -1.0;

  auto rhsBlockBase = [&](int t) {
    return rhs->data() + static_cast<size_t>(t) * rhsBlockStride;
  };

  // Forward: w_0 = L_00^{-1} b_0; w_t = L_tt^{-1}(b_t - Z_t^T w_{t-1}).
  // GEMM goes strided-batched (see factorize()'s comment on why TRSM
  // cannot); TRSM still builds a pointer array per step via
  // uploadBatchPointers.
  uploadBatchPointers(impl_->diagonalBlockBase(0), batchSize, diagStride,
                     stream, &impl_->ptrsA);
  uploadBatchPointers(rhsBlockBase(0), batchSize, rhsBatchStride, stream,
                     &impl_->ptrsB);
  GTSAM_CUBLAS_CHECK(cublasDtrsmBatched(
      impl_->blasHandle, CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER, CUBLAS_OP_N,
      CUBLAS_DIAG_NON_UNIT, blockDim, k, &one, impl_->ptrsA.data(), blockDim,
      impl_->ptrsB.data(), blockDim, batchSize));

  for (int t = 1; t < numBlocks; ++t) {
    // b_t updated to b_t - Z_t^T w_{t-1}, then solved in place to become w_t.
    GTSAM_CUBLAS_CHECK(cublasDgemmStridedBatched(
        impl_->blasHandle, CUBLAS_OP_T, CUBLAS_OP_N, blockDim, k, blockDim,
        &minusOne, impl_->offDiagonalBlockBase(t - 1), blockDim,
        static_cast<long long>(offDiagStride), rhsBlockBase(t - 1), blockDim,
        static_cast<long long>(rhsBatchStride), &one, rhsBlockBase(t),
        blockDim, static_cast<long long>(rhsBatchStride), batchSize));

    uploadBatchPointers(impl_->diagonalBlockBase(t), batchSize, diagStride,
                       stream, &impl_->ptrsA);
    uploadBatchPointers(rhsBlockBase(t), batchSize, rhsBatchStride, stream,
                       &impl_->ptrsB);
    GTSAM_CUBLAS_CHECK(cublasDtrsmBatched(
        impl_->blasHandle, CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
        CUBLAS_OP_N, CUBLAS_DIAG_NON_UNIT, blockDim, k, &one,
        impl_->ptrsA.data(), blockDim, impl_->ptrsB.data(), blockDim,
        batchSize));
  }

  // Backward: x_{T-1} = L_{T-1,T-1}^{-T} w_{T-1};
  //           x_t = L_tt^{-T}(w_t - Z_{t+1} x_{t+1}).
  uploadBatchPointers(impl_->diagonalBlockBase(numBlocks - 1), batchSize,
                     diagStride, stream, &impl_->ptrsA);
  uploadBatchPointers(rhsBlockBase(numBlocks - 1), batchSize, rhsBatchStride,
                     stream, &impl_->ptrsB);
  GTSAM_CUBLAS_CHECK(cublasDtrsmBatched(
      impl_->blasHandle, CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER, CUBLAS_OP_T,
      CUBLAS_DIAG_NON_UNIT, blockDim, k, &one, impl_->ptrsA.data(), blockDim,
      impl_->ptrsB.data(), blockDim, batchSize));

  for (int t = numBlocks - 2; t >= 0; --t) {
    // w_t updated to w_t - Z_{t+1} x_{t+1} (Z_{t+1} not transposed this
    // time), then solved in place to become x_t.
    GTSAM_CUBLAS_CHECK(cublasDgemmStridedBatched(
        impl_->blasHandle, CUBLAS_OP_N, CUBLAS_OP_N, blockDim, k, blockDim,
        &minusOne, impl_->offDiagonalBlockBase(t), blockDim,
        static_cast<long long>(offDiagStride), rhsBlockBase(t + 1), blockDim,
        static_cast<long long>(rhsBatchStride), &one, rhsBlockBase(t),
        blockDim, static_cast<long long>(rhsBatchStride), batchSize));

    uploadBatchPointers(impl_->diagonalBlockBase(t), batchSize, diagStride,
                       stream, &impl_->ptrsA);
    uploadBatchPointers(rhsBlockBase(t), batchSize, rhsBatchStride, stream,
                       &impl_->ptrsB);
    GTSAM_CUBLAS_CHECK(cublasDtrsmBatched(
        impl_->blasHandle, CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_LOWER,
        CUBLAS_OP_T, CUBLAS_DIAG_NON_UNIT, blockDim, k, &one,
        impl_->ptrsA.data(), blockDim, impl_->ptrsB.data(), blockDim,
        batchSize));
  }

  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  if (stats) {
    stats->backend = LinearSolverType::DenseCholesky;
    stats->solveCount += static_cast<size_t>(k);
    stats->solveSeconds += elapsed(start);
  }
}

int BatchedBlockTridiagonalCholeskySolver::batchSize() const {
  return impl_->batchSize;
}
int BatchedBlockTridiagonalCholeskySolver::numBlocks() const {
  return impl_->numBlocks;
}
int BatchedBlockTridiagonalCholeskySolver::blockDim() const {
  return impl_->blockDim;
}

}  // namespace gtsam::cuda
