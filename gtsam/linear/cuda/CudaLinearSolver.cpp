#include <gtsam/linear/cuda/CudaLinearSolver.h>

#include <gtsam/linear/cuda/CudaDenseCholeskySolver.h>
#include <gtsam/linear/cuda/CudaPcgSolver.h>
#include <gtsam/linear/cuda/CudssSpdSolver.h>

#include <memory>
#include <stdexcept>

namespace gtsam::cuda {

struct CudaLinearSolverSession::Impl {
  explicit Impl(const CudaLinearSolverOptions& optionsIn)
      : options(optionsIn) {
    switch (options.backend) {
      case CudaLinearSolverType::DenseCholesky:
        dense = std::make_unique<CudaDenseCholeskySolver>();
        break;
      case CudaLinearSolverType::Cudss:
        cudss = std::make_unique<CudssSpdSolver>();
        break;
      case CudaLinearSolverType::Pcg:
        pcg = std::make_unique<CudaPcgSolver>();
        break;
    }
    localStats.backend = options.backend;
  }

  CudaLinearSolverOptions options;
  std::unique_ptr<CudaDenseCholeskySolver> dense;
  std::unique_ptr<CudssSpdSolver> cudss;
  std::unique_ptr<CudaPcgSolver> pcg;
  CudaLinearSolveStats localStats;
};

CudaLinearSolverSession::CudaLinearSolverSession(
    const CudaLinearSolverOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
CudaLinearSolverSession::~CudaLinearSolverSession() = default;
CudaLinearSolverSession::CudaLinearSolverSession(
    CudaLinearSolverSession&&) noexcept = default;
CudaLinearSolverSession& CudaLinearSolverSession::operator=(
    CudaLinearSolverSession&&) noexcept = default;

bool CudaLinearSolverSession::Supports(CudaLinearSolverType backend,
                                       CudaLinearSystemKind systemKind) {
  switch (backend) {
    case CudaLinearSolverType::DenseCholesky:
      return systemKind == CudaLinearSystemKind::Dense;
    case CudaLinearSolverType::Cudss:
      return systemKind == CudaLinearSystemKind::Sparse;
    case CudaLinearSolverType::Pcg:
      return systemKind == CudaLinearSystemKind::Operator;
  }
  return false;
}

void CudaLinearSolverSession::Validate(
    const CudaLinearSolverOptions& options,
    CudaLinearSystemKind systemKind) {
  if (!Supports(options.backend, systemKind)) {
    throw std::invalid_argument(
        "CUDA linear solver does not support the requested system kind");
  }
  if (options.useUserOrdering &&
      options.backend != CudaLinearSolverType::Cudss) {
    throw std::invalid_argument(
        "CUDA user ordering is supported only by cuDSS");
  }
}

void CudaLinearSolverSession::analyze(int denseDimension,
                                      cudaStream_t stream) {
  Validate(impl_->options, CudaLinearSystemKind::Dense);
  impl_->dense->analyze(denseDimension, stream);
  ++impl_->localStats.analysisCount;
}

void CudaLinearSolverSession::analyze(
    const DeviceSparseSpdSystem& system,
    CudaDeviceArray<double>* solution, cudaStream_t stream) {
  Validate(impl_->options, CudaLinearSystemKind::Sparse);
  if (impl_->options.useUserOrdering) {
    throw std::invalid_argument(
        "CUDA linear session requires the configured user ordering");
  }
  impl_->cudss->analyze(system, solution, stream);
}

void CudaLinearSolverSession::analyze(
    const DeviceSparseSpdSystem& system,
    CudaDeviceArray<double>* solution,
    const std::vector<int>& scalarPermutation, cudaStream_t stream) {
  Validate(impl_->options, CudaLinearSystemKind::Sparse);
  if (!impl_->options.useUserOrdering) {
    throw std::invalid_argument(
        "CUDA linear session received an ordering in automatic mode");
  }
  impl_->cudss->analyze(system, solution, scalarPermutation, stream);
}

void CudaLinearSolverSession::analyze(int operatorDimension,
                                      const CudaPcgOptions& pcgOptions,
                                      cudaStream_t stream,
                                      bool collectProfile) {
  Validate(impl_->options, CudaLinearSystemKind::Operator);
  impl_->pcg->initialize(operatorDimension, pcgOptions, stream,
                         collectProfile);
}

void CudaLinearSolverSession::solve(CudaDenseSpdSystemView system,
                                    CudaDeviceArray<double>* solution,
                                    cudaStream_t stream) {
  Validate(impl_->options, CudaLinearSystemKind::Dense);
  impl_->dense->solve(system, solution, stream, &impl_->localStats);
}

void CudaLinearSolverSession::solve(const DeviceSparseSpdSystem& system,
                                    CudaDeviceArray<double>* solution,
                                    cudaStream_t stream) {
  Validate(impl_->options, CudaLinearSystemKind::Sparse);
  impl_->cudss->solve(system, solution, stream);
}

void CudaLinearSolverSession::solve(
    const CudaLinearOperator& linearOperator,
    const CudaPreconditioner& preconditioner, const double* rhs,
    CudaDeviceArray<double>* solution, cudaStream_t stream) {
  Validate(impl_->options, CudaLinearSystemKind::Operator);
  impl_->pcg->solve(linearOperator, preconditioner, rhs, solution, stream);
}

void CudaLinearSolverSession::invalidateWarmStart() {
  if (impl_->pcg) impl_->pcg->invalidateWarmStart();
}

const CudaLinearSolveStats& CudaLinearSolverSession::stats() const {
  switch (impl_->options.backend) {
    case CudaLinearSolverType::DenseCholesky:
      return impl_->localStats;
    case CudaLinearSolverType::Cudss:
      return impl_->cudss->stats();
    case CudaLinearSolverType::Pcg:
      return impl_->pcg->stats();
  }
  throw std::logic_error("CUDA linear session has unknown backend");
}

}  // namespace gtsam::cuda
