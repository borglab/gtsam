/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockHessianAssembler.cpp
 * @brief   Batched GPU formation of block-tridiagonal normal-equation
 *          pieces from B dense Jacobians, via cuBLAS batched GEMM
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <gtsam/linear/cuda/internal/BatchedBlockHessianAssembler.h>

#include <gtsam/base/cuda/Errors.h>

#include <cublas_v2.h>
#include <sstream>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

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

#define GTSAM_CUBLAS_CHECK(expr) checkCublas((expr), #expr)

}  // namespace

struct BatchedBlockHessianAssembler::Impl {
  cublasHandle_t blasHandle = nullptr;
  // Raw At^T*At+1 results before discarding the one invalid cross-item
  // pair per item (see assemble()'s offDiag comment).
  DeviceArray<double> rawOffDiag;
  // -1 means "whatever device is already current" (see
  // BatchedDenseSolverOptions); otherwise re-asserted at the top of
  // assemble() too, not just here at construction -- cudaSetDevice is
  // thread-global, so a second same-thread instance constructed on a
  // different device would otherwise silently leave this one's later calls
  // running against the wrong device's memory.
  int deviceId = -1;

  explicit Impl(const BatchedDenseSolverOptions& options)
      : deviceId(options.deviceId) {
    if (deviceId >= 0) {
      GTSAM_CUDA_CHECK(cudaSetDevice(deviceId));
    }
    GTSAM_CUBLAS_CHECK(cublasCreate(&blasHandle));
  }
  ~Impl() {
    if (blasHandle) cublasDestroy(blasHandle);
  }
};

BatchedBlockHessianAssembler::BatchedBlockHessianAssembler(
    const BatchedDenseSolverOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
BatchedBlockHessianAssembler::~BatchedBlockHessianAssembler() = default;
BatchedBlockHessianAssembler::BatchedBlockHessianAssembler(
    BatchedBlockHessianAssembler&&) noexcept = default;
BatchedBlockHessianAssembler& BatchedBlockHessianAssembler::operator=(
    BatchedBlockHessianAssembler&&) noexcept = default;

void BatchedBlockHessianAssembler::assemble(
    int batchSize, int numBlocks, int blockDim, int jacobianRows,
    DeviceArray<double>* deviceA, DeviceArray<double>* deviceB,
    DeviceArray<double>* diagOut, DeviceArray<double>* offDiagOut,
    DeviceArray<double>* rhsOut, cudaStream_t stream) {
  if (batchSize <= 0 || numBlocks <= 1 || blockDim <= 0 || jacobianRows <= 0 ||
      !deviceA || !deviceB || !diagOut || !offDiagOut || !rhsOut) {
    throw std::invalid_argument(
        "BatchedBlockHessianAssembler::assemble requires numBlocks > 1 and "
        "non-null buffers");
  }
  if (impl_->deviceId >= 0) {
    GTSAM_CUDA_CHECK(cudaSetDevice(impl_->deviceId));
  }
  const size_t dimension = static_cast<size_t>(numBlocks) * blockDim;
  const size_t expectedA =
      static_cast<size_t>(batchSize) * jacobianRows * dimension;
  const size_t expectedB = static_cast<size_t>(batchSize) * jacobianRows;
  if (deviceA->size() != expectedA || deviceB->size() != expectedB) {
    throw std::invalid_argument(
        "BatchedBlockHessianAssembler::assemble: deviceA/deviceB size must "
        "be batchSize*jacobianRows*(numBlocks*blockDim)/batchSize*"
        "jacobianRows -- every item must share the same residual count");
  }
  diagOut->resize(static_cast<size_t>(batchSize) * numBlocks * blockDim *
                  blockDim);
  offDiagOut->resize(static_cast<size_t>(batchSize) * (numBlocks - 1) *
                     blockDim * blockDim);
  rhsOut->resize(static_cast<size_t>(batchSize) * dimension);

  GTSAM_CUBLAS_CHECK(cublasSetStream(impl_->blasHandle, stream));
  const double one = 1.0;
  const double zero = 0.0;
  const long long blockStride =
      static_cast<long long>(blockDim) * jacobianRows;

  // diag[i][t] = At^T At, every (item, block) in one batched GEMM: the
  // whole deviceA buffer is one uniformly-strided block sequence.
  GTSAM_CUBLAS_CHECK(cublasDgemmStridedBatched(
      impl_->blasHandle, CUBLAS_OP_T, CUBLAS_OP_N, blockDim, blockDim,
      jacobianRows, &one, deviceA->data(), jacobianRows, blockStride,
      deviceA->data(), jacobianRows, blockStride, &zero, diagOut->data(),
      blockDim, static_cast<long long>(blockDim) * blockDim,
      batchSize * numBlocks));

  // rhs[i] = A_i^T b_i, flattened to dimension entries directly (one
  // batched GEMM over items, no per-block split needed).
  GTSAM_CUBLAS_CHECK(cublasDgemmStridedBatched(
      impl_->blasHandle, CUBLAS_OP_T, CUBLAS_OP_N,
      static_cast<int>(dimension), 1, jacobianRows, &one, deviceA->data(),
      jacobianRows, static_cast<long long>(jacobianRows) * dimension,
      deviceB->data(), jacobianRows, static_cast<long long>(jacobianRows),
      &zero, rhsOut->data(), static_cast<int>(dimension),
      static_cast<long long>(dimension), batchSize));

  // offDiag[i][t] = At^T At+1: one GEMM over all consecutive block pairs
  // also computes one invalid cross-item pair per boundary, discarded below.
  const int rawPairs = batchSize * numBlocks - 1;
  impl_->rawOffDiag.resize(static_cast<size_t>(rawPairs) * blockDim *
                           blockDim);
  GTSAM_CUBLAS_CHECK(cublasDgemmStridedBatched(
      impl_->blasHandle, CUBLAS_OP_T, CUBLAS_OP_N, blockDim, blockDim,
      jacobianRows, &one, deviceA->data(), jacobianRows, blockStride,
      deviceA->data() + static_cast<size_t>(blockDim) * jacobianRows,
      jacobianRows, blockStride, &zero, impl_->rawOffDiag.data(), blockDim,
      static_cast<long long>(blockDim) * blockDim, rawPairs));

  const size_t blockBytes =
      static_cast<size_t>(blockDim) * blockDim * sizeof(double);
  GTSAM_CUDA_CHECK(cudaMemcpy2DAsync(
      offDiagOut->data(), (numBlocks - 1) * blockBytes,
      impl_->rawOffDiag.data(), static_cast<size_t>(numBlocks) * blockBytes,
      (numBlocks - 1) * blockBytes, static_cast<size_t>(batchSize),
      cudaMemcpyDeviceToDevice, stream));
}

}  // namespace gtsam::cuda
