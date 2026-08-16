#include <gtsam/linear/cuda/CudaDenseCholeskySolver.h>

#include <gtsam/base/cuda/CudaErrors.h>

#include <algorithm>
#include <chrono>
#include <cusolverDn.h>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {
namespace {

const char* CusolverStatusName(cusolverStatus_t status) {
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

void CheckCusolver(cusolverStatus_t status, const char* expression) {
  if (status == CUSOLVER_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuSOLVER call failed: " << expression << " returned "
     << CusolverStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

#define GTSAM_CUSOLVER_CHECK(expr) CheckCusolver((expr), #expr)

void CheckDeviceInfo(const CudaDeviceArray<int>& info, cudaStream_t stream,
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

double Elapsed(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      start)
      .count();
}

}  // namespace

struct CudaDenseCholeskySolver::Impl {
  cusolverDnHandle_t handle = nullptr;
  CudaDeviceArray<double> workspace;
  CudaDeviceArray<int> info;
  int analyzedDimension = 0;

  Impl() { GTSAM_CUSOLVER_CHECK(cusolverDnCreate(&handle)); }
  ~Impl() {
    if (handle) cusolverDnDestroy(handle);
  }
};

CudaDenseCholeskySolver::CudaDenseCholeskySolver()
    : impl_(std::make_unique<Impl>()) {}
CudaDenseCholeskySolver::~CudaDenseCholeskySolver() = default;
CudaDenseCholeskySolver::CudaDenseCholeskySolver(
    CudaDenseCholeskySolver&&) noexcept = default;
CudaDenseCholeskySolver& CudaDenseCholeskySolver::operator=(
    CudaDenseCholeskySolver&&) noexcept = default;

void CudaDenseCholeskySolver::analyze(int maximumDimension,
                                      cudaStream_t stream) {
  if (maximumDimension < 0) {
    throw std::invalid_argument(
        "CudaDenseCholeskySolver dimension must be nonnegative");
  }
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(impl_->handle, stream));
  impl_->analyzedDimension = maximumDimension;
}

void CudaDenseCholeskySolver::solveInPlace(CudaDenseSpdSystemView system,
                                           cudaStream_t stream,
                                           CudaLinearSolveStats* stats) {
  if (system.dimension <= 0 || system.leadingDimension < system.dimension ||
      !system.values || !system.rhs) {
    throw std::invalid_argument(
        "CudaDenseCholeskySolver requires a nonempty valid dense system");
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
  CheckDeviceInfo(impl_->info, stream, "cusolverDnDpotrf");
  const double factorizationSeconds = Elapsed(factorizationStart);

  const auto solveStart = std::chrono::steady_clock::now();
  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrs(
      impl_->handle, CUBLAS_FILL_MODE_LOWER, system.dimension, 1,
      system.values, system.leadingDimension, system.rhs, system.dimension,
      impl_->info.data()));
  CheckDeviceInfo(impl_->info, stream, "cusolverDnDpotrs");
  const double solveSeconds = Elapsed(solveStart);

  impl_->analyzedDimension =
      std::max(impl_->analyzedDimension, system.dimension);
  if (stats) {
    stats->backend = CudaLinearSolverType::DenseCholesky;
    ++stats->factorizationCount;
    ++stats->solveCount;
    stats->factorizationSeconds += factorizationSeconds;
    stats->solveSeconds += solveSeconds;
  }
}

void CudaDenseCholeskySolver::solve(CudaDenseSpdSystemView system,
                                    CudaDeviceArray<double>* solution,
                                    cudaStream_t stream,
                                    CudaLinearSolveStats* stats) {
  if (!solution) {
    throw std::invalid_argument(
        "CudaDenseCholeskySolver requires solution output");
  }
  solveInPlace(system, stream, stats);
  solution->resize(static_cast<size_t>(system.dimension));
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      solution->data(), system.rhs,
      sizeof(double) * static_cast<size_t>(system.dimension),
      cudaMemcpyDeviceToDevice, stream));
}

}  // namespace gtsam::cuda
