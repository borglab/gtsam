#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSystem.h>

#include <cstddef>

namespace gtsam::cuda {

enum class CudaLinearSolverType { DenseCholesky, Cudss, Pcg };

struct CudaLinearSolverOptions {
  CudaLinearSolverType backend = CudaLinearSolverType::Cudss;
  bool useUserOrdering = false;
};

struct CudaLinearSolveStats {
  CudaLinearSolverType backend = CudaLinearSolverType::Cudss;
  bool userOrderingApplied = false;
  size_t analysisCount = 0;
  size_t factorizationCount = 0;
  size_t solveCount = 0;
  size_t pcgIterationsTotal = 0;
  size_t pcgMaxIterationHits = 0;
  bool lastPcgConverged = false;
  double analysisSeconds = 0.0;
  double factorizationSeconds = 0.0;
  double solveSeconds = 0.0;
  double preconditionerSeconds = 0.0;
};

class GTSAM_EXPORT CudaLinearSolverSession {
 public:
  static bool Supports(CudaLinearSolverType backend,
                       CudaLinearSystemKind systemKind);
  static void Validate(const CudaLinearSolverOptions& options,
                       CudaLinearSystemKind systemKind);
};

}  // namespace gtsam::cuda
