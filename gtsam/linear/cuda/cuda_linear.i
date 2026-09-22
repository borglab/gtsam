//*************************************************************************
// Shared CUDA linear solver configuration
//*************************************************************************

namespace gtsam {
namespace cuda {

#include <gtsam/linear/cuda/LinearSolver.h>

enum class LinearSolverType {
  DenseCholesky,
  Cudss,
  Pcg
};

class LinearSolverOptions {
  LinearSolverOptions();

  gtsam::cuda::LinearSolverType backend;
};

class PcgOptions {
  PcgOptions();

  int maxIterations;
  double relativeTolerance;
  bool warmStart;
  int convergenceCheckInterval;
};

}  // namespace cuda
}  // namespace gtsam
