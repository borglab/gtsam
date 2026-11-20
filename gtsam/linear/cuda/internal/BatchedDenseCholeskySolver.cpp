/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedDenseCholeskySolver.cpp
 * @brief   Persistent cuSOLVER DN batched Cholesky backend for B independent
 *          dense SPD systems of the same dimension
 * @date    Nov 20, 2026
 */

#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>

#include <gtsam/base/cuda/Errors.h>

#include <chrono>
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

#define GTSAM_CUSOLVER_CHECK(expr) checkCusolver((expr), #expr)

double elapsed(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       start)
      .count();
}

/// Fills a persistent device pointer array (one pointer per batch item, base
/// + i * strideElements) by overwriting `cache` in place. `cache` is resized
/// only the first time (or if batchSize changes); every later call just
/// re-uploads the small pointer values, with no new cudaMalloc/cudaFree --
/// profiling an earlier version of this class found repeated allocation
/// (a fresh DeviceArray per factorize()/solve() call) to be the dominant real
/// cost at DFG-TAMP's problem size, well above the actual cuSOLVER compute.
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

struct BatchedDenseCholeskySolver::Impl {
  cusolverDnHandle_t handle = nullptr;
  // Persistent across calls so factorize()/solve() never cudaMalloc/cudaFree
  // on the steady-state path -- only resize() the first time, or if
  // batchSize/k actually changes, does either array reallocate.
  DeviceArray<double*> aPointers;
  DeviceArray<double*> bPointers;
  DeviceArray<int> potrfInfo;
  DeviceArray<int> potrsInfo;
  double* valuesBase = nullptr;
  int batchSize = 0;
  int dimension = 0;
  bool factorized = false;

  explicit Impl(const BatchedDenseSolverOptions& options) {
    if (options.deviceId >= 0) {
      GTSAM_CUDA_CHECK(cudaSetDevice(options.deviceId));
    }
    GTSAM_CUSOLVER_CHECK(cusolverDnCreate(&handle));
  }
  ~Impl() {
    if (handle) cusolverDnDestroy(handle);
  }
};

BatchedDenseCholeskySolver::BatchedDenseCholeskySolver(
    const BatchedDenseSolverOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
BatchedDenseCholeskySolver::~BatchedDenseCholeskySolver() = default;
BatchedDenseCholeskySolver::BatchedDenseCholeskySolver(
    BatchedDenseCholeskySolver&&) noexcept = default;
BatchedDenseCholeskySolver& BatchedDenseCholeskySolver::operator=(
    BatchedDenseCholeskySolver&&) noexcept = default;

void BatchedDenseCholeskySolver::factorize(int batchSize, int dimension,
                                          DeviceArray<double>* values,
                                          cudaStream_t stream,
                                          LinearSolveStats* stats) {
  if (batchSize <= 0 || dimension <= 0 || !values ||
      values->size() != static_cast<size_t>(batchSize) *
                             static_cast<size_t>(dimension) *
                             static_cast<size_t>(dimension)) {
    throw std::invalid_argument(
        "BatchedDenseCholeskySolver::factorize requires a nonempty batch "
        "matching batchSize*dimension*dimension");
  }
  const auto start = std::chrono::steady_clock::now();
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(impl_->handle, stream));

  const size_t stride =
      static_cast<size_t>(dimension) * static_cast<size_t>(dimension);
  uploadBatchPointers(values->data(), batchSize, stride, stream,
                     &impl_->aPointers);
  impl_->potrfInfo.resize(static_cast<size_t>(batchSize));

  // Queued, not synchronized: the caller (solve(), right after this call on
  // the same stream) does not need factorization to be host-visible before
  // it can enqueue more work -- GPU-side stream ordering already guarantees
  // solve()'s kernels wait for this one. The single synchronization point
  // that actually matters is deferred to right before info is read back,
  // below -- one blocking round trip per factorize() instead of two.
  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrfBatched(
      impl_->handle, CUBLAS_FILL_MODE_LOWER, dimension, impl_->aPointers.data(),
      dimension, impl_->potrfInfo.data(), batchSize));

  std::vector<int> hostInfo;
  impl_->potrfInfo.download(&hostInfo, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  for (int i = 0; i < batchSize; ++i) {
    if (hostInfo[static_cast<size_t>(i)] != 0) {
      std::ostringstream os;
      os << "cusolverDnDpotrfBatched: batch item " << i
         << " is not positive definite (info="
         << hostInfo[static_cast<size_t>(i)] << ")";
      throw std::runtime_error(os.str());
    }
  }

  impl_->valuesBase = values->data();
  impl_->batchSize = batchSize;
  impl_->dimension = dimension;
  impl_->factorized = true;

  if (stats) {
    stats->backend = LinearSolverType::DenseCholesky;
    ++stats->factorizationCount;
    stats->factorizationSeconds += elapsed(start);
  }
}

void BatchedDenseCholeskySolver::solve(int k, DeviceArray<double>* rhs,
                                      cudaStream_t stream,
                                      LinearSolveStats* stats) {
  if (!impl_->factorized) {
    throw std::logic_error(
        "BatchedDenseCholeskySolver::solve called before factorize");
  }
  if (k <= 0 || !rhs ||
      rhs->size() != static_cast<size_t>(impl_->batchSize) *
                         static_cast<size_t>(impl_->dimension) *
                         static_cast<size_t>(k)) {
    throw std::invalid_argument(
        "BatchedDenseCholeskySolver::solve requires rhs sized "
        "batchSize*dimension*k matching the factorized batch");
  }
  const auto start = std::chrono::steady_clock::now();
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(impl_->handle, stream));

  // aPointers already holds this exact factorization's pointers from the
  // matching factorize() call -- no re-upload needed, it hasn't changed.
  const size_t bItemStride =
      static_cast<size_t>(impl_->dimension) * static_cast<size_t>(k);
  // One info slot per column so none of the k calls below need to wait for
  // the others before enqueuing -- cusolverDnXpotrsBatched accepts only
  // nrhs=1 per call (confirmed empirically: CUSOLVER_STATUS_INVALID_VALUE
  // for nrhs>1), so k right-hand-side columns means k calls, every one
  // reusing the same factorization with no refactorization, but none of
  // them need a synchronization barrier between them: they are independent
  // column solves against the same fixed factorization, and GPU-side stream
  // ordering already serializes them correctly without a host round trip.
  impl_->potrsInfo.resize(static_cast<size_t>(k));
  for (int column = 0; column < k; ++column) {
    uploadBatchPointers(
        rhs->data() + static_cast<size_t>(column) *
                          static_cast<size_t>(impl_->dimension),
        impl_->batchSize, bItemStride, stream, &impl_->bPointers);
    GTSAM_CUSOLVER_CHECK(cusolverDnDpotrsBatched(
        impl_->handle, CUBLAS_FILL_MODE_LOWER, impl_->dimension, /*nrhs=*/1,
        impl_->aPointers.data(), impl_->dimension, impl_->bPointers.data(),
        impl_->dimension, impl_->potrsInfo.data() + column, impl_->batchSize));
  }

  // The one synchronization point for this whole solve() call: waits for
  // every column's kernel (so the numeric result in `rhs` is also complete,
  // not just the info download) and makes every column's info host-visible
  // at once, instead of one blocking round trip per column.
  std::vector<int> hostInfo;
  impl_->potrsInfo.download(&hostInfo, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  for (int column = 0; column < k; ++column) {
    if (hostInfo[static_cast<size_t>(column)] != 0) {
      std::ostringstream os;
      os << "cusolverDnDpotrsBatched failed on rhs column " << column
         << " with info=" << hostInfo[static_cast<size_t>(column)];
      throw std::runtime_error(os.str());
    }
  }

  if (stats) {
    stats->backend = LinearSolverType::DenseCholesky;
    stats->solveCount += static_cast<size_t>(k);
    stats->solveSeconds += elapsed(start);
  }
}

int BatchedDenseCholeskySolver::batchSize() const { return impl_->batchSize; }
int BatchedDenseCholeskySolver::dimension() const { return impl_->dimension; }

}  // namespace gtsam::cuda
