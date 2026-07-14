#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>

#include <memory>

namespace gtsam::cuda {

/** Host wall time spent only in cuDSS's mandatory DATA_INFO boundary. */
struct CudssSpdSolveProfile {
  double dataInfoBoundaryWall = 0.0;
};

class GTSAM_EXPORT CudssSpdSolver {
 public:
  CudssSpdSolver();
  ~CudssSpdSolver();

  CudssSpdSolver(const CudssSpdSolver&) = delete;
  CudssSpdSolver& operator=(const CudssSpdSolver&) = delete;
  CudssSpdSolver(CudssSpdSolver&&) noexcept;
  CudssSpdSolver& operator=(CudssSpdSolver&&) noexcept;

  void analyze(const DeviceSparseNormalEquations& system,
               CudaDeviceArray<double>* solution,
               cudaStream_t stream = nullptr);
  /**
   * Numerically factor and solve the analyzed SPD system.
   *
   * Throws std::runtime_error when cuDSS reports a non-positive minor during
   * numerical factorization.
   */
  void solve(const DeviceSparseNormalEquations& system,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr,
             CudssSpdSolveProfile* profile = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class GTSAM_EXPORT CudssLinearSolver {
 public:
  void solveSpd(const DeviceSparseNormalEquations& system,
                CudaDeviceArray<double>* solution,
                cudaStream_t stream = nullptr) const;
};

}  // namespace gtsam::cuda
