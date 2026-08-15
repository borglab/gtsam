#include <gtsam/linear/cuda/CudaLinearSolver.h>

#include <stdexcept>

namespace gtsam::cuda {

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

}  // namespace gtsam::cuda
