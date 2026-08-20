/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DenseCholeskySolver.cpp
 * @brief   Persistent cuSOLVER DN Cholesky backend for dense SPD systems
 * @author  Ruogu Li
 * @date    Aug 15, 2026
 */

#include <gtsam/linear/cuda/internal/DenseCholeskySolver.h>

#include <gtsam/base/cuda/Errors.h>

#include <algorithm>
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
    case CUSOLVER_STATUS_MAPPING_ERROR:
      return "CUSOLVER_STATUS_MAPPING_ERROR";
    case CUSOLVER_STATUS_EXECUTION_FAILED:
      return "CUSOLVER_STATUS_EXECUTION_FAILED";
    case CUSOLVER_STATUS_INTERNAL_ERROR:
      return "CUSOLVER_STATUS_INTERNAL_ERROR";
    case CUSOLVER_STATUS_MATRIX_TYPE_NOT_SUPPORTED:
      return "CUSOLVER_STATUS_MATRIX_TYPE_NOT_SUPPORTED";
    case CUSOLVER_STATUS_NOT_SUPPORTED:
      return "CUSOLVER_STATUS_NOT_SUPPORTED";
    case CUSOLVER_STATUS_ZERO_PIVOT:
      return "CUSOLVER_STATUS_ZERO_PIVOT";
    case CUSOLVER_STATUS_INVALID_LICENSE:
      return "CUSOLVER_STATUS_INVALID_LICENSE";
    case CUSOLVER_STATUS_IRS_PARAMS_NOT_INITIALIZED:
      return "CUSOLVER_STATUS_IRS_PARAMS_NOT_INITIALIZED";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID_PREC:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID_PREC";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID_REFINE:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID_REFINE";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID_MAXITER:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID_MAXITER";
    case CUSOLVER_STATUS_IRS_INTERNAL_ERROR:
      return "CUSOLVER_STATUS_IRS_INTERNAL_ERROR";
    case CUSOLVER_STATUS_IRS_NOT_SUPPORTED:
      return "CUSOLVER_STATUS_IRS_NOT_SUPPORTED";
    case CUSOLVER_STATUS_IRS_OUT_OF_RANGE:
      return "CUSOLVER_STATUS_IRS_OUT_OF_RANGE";
    case CUSOLVER_STATUS_IRS_NRHS_NOT_SUPPORTED_FOR_REFINE_GMRES:
      return "CUSOLVER_STATUS_IRS_NRHS_NOT_SUPPORTED_FOR_REFINE_GMRES";
    case CUSOLVER_STATUS_IRS_INFOS_NOT_INITIALIZED:
      return "CUSOLVER_STATUS_IRS_INFOS_NOT_INITIALIZED";
    case CUSOLVER_STATUS_IRS_INFOS_NOT_DESTROYED:
      return "CUSOLVER_STATUS_IRS_INFOS_NOT_DESTROYED";
    case CUSOLVER_STATUS_IRS_MATRIX_SINGULAR:
      return "CUSOLVER_STATUS_IRS_MATRIX_SINGULAR";
    case CUSOLVER_STATUS_INVALID_WORKSPACE:
      return "CUSOLVER_STATUS_INVALID_WORKSPACE";
  }
  return "CUSOLVER_STATUS_UNKNOWN";
}

void checkCusolver(cusolverStatus_t status, const char* expression) {
  if (status == CUSOLVER_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuSOLVER call failed: " << expression << " returned "
     << cusolverStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

#define GTSAM_CUSOLVER_CHECK(expr) checkCusolver((expr), #expr)

void checkDeviceInfo(const DeviceArray<int>& info, cudaStream_t stream,
                     const char* operation) {
  std::vector<int> hostInfo;
  info.download(&hostInfo, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  if (!hostInfo.empty() && hostInfo[0] != 0) {
    std::ostringstream os;
    os << operation << " failed with device info " << hostInfo[0];
    throw std::runtime_error(os.str());
  }
}

double elapsed(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      start)
      .count();
}

}  // namespace

struct DenseCholeskySolver::Impl {
  cusolverDnHandle_t handle = nullptr;
  DeviceArray<double> workspace;
  DeviceArray<int> info;
  int analyzedDimension = 0;

  Impl() { GTSAM_CUSOLVER_CHECK(cusolverDnCreate(&handle)); }
  ~Impl() {
    if (handle) cusolverDnDestroy(handle);
  }
};

DenseCholeskySolver::DenseCholeskySolver()
    : impl_(std::make_unique<Impl>()) {}
DenseCholeskySolver::~DenseCholeskySolver() = default;
DenseCholeskySolver::DenseCholeskySolver(
    DenseCholeskySolver&&) noexcept = default;
DenseCholeskySolver& DenseCholeskySolver::operator=(
    DenseCholeskySolver&&) noexcept = default;

void DenseCholeskySolver::analyze(int maximumDimension,
                                      cudaStream_t stream) {
  if (maximumDimension < 0) {
    throw std::invalid_argument(
        "DenseCholeskySolver dimension must be nonnegative");
  }
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(impl_->handle, stream));
  impl_->analyzedDimension = maximumDimension;
}

void DenseCholeskySolver::solveInPlace(DenseSpdSystemView system,
                                           cudaStream_t stream,
                                           LinearSolveStats* stats) {
  if (system.dimension <= 0 || system.leadingDimension < system.dimension ||
      !system.values || !system.rhs) {
    throw std::invalid_argument(
        "DenseCholeskySolver requires a nonempty valid dense system");
  }
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(impl_->handle, stream));
  int workspaceSize = 0;
  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrf_bufferSize(
      impl_->handle, CUBLAS_FILL_MODE_LOWER, system.dimension, system.values,
      system.leadingDimension, &workspaceSize));
  impl_->workspace.resize(static_cast<size_t>(workspaceSize));
  impl_->info.resize(1);

  const auto factorizationStart = std::chrono::steady_clock::now();
  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrf(
      impl_->handle, CUBLAS_FILL_MODE_LOWER, system.dimension, system.values,
      system.leadingDimension, impl_->workspace.data(), workspaceSize,
      impl_->info.data()));
  checkDeviceInfo(impl_->info, stream, "cusolverDnDpotrf");
  const double factorizationSeconds = elapsed(factorizationStart);

  const auto solveStart = std::chrono::steady_clock::now();
  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrs(
      impl_->handle, CUBLAS_FILL_MODE_LOWER, system.dimension, 1,
      system.values, system.leadingDimension, system.rhs, system.dimension,
      impl_->info.data()));
  checkDeviceInfo(impl_->info, stream, "cusolverDnDpotrs");
  const double solveSeconds = elapsed(solveStart);

  impl_->analyzedDimension =
      std::max(impl_->analyzedDimension, system.dimension);
  if (stats) {
    stats->backend = LinearSolverType::DenseCholesky;
    ++stats->factorizationCount;
    ++stats->solveCount;
    stats->factorizationSeconds += factorizationSeconds;
    stats->solveSeconds += solveSeconds;
  }
}

void DenseCholeskySolver::solve(DenseSpdSystemView system,
                                    DeviceArray<double>* solution,
                                    cudaStream_t stream,
                                    LinearSolveStats* stats) {
  if (!solution) {
    throw std::invalid_argument(
        "DenseCholeskySolver requires solution output");
  }
  solveInPlace(system, stream, stats);
  solution->resize(static_cast<size_t>(system.dimension));
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      solution->data(), system.rhs,
      sizeof(double) * static_cast<size_t>(system.dimension),
      cudaMemcpyDeviceToDevice, stream));
}

}  // namespace gtsam::cuda
