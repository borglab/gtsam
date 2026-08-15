#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

#include <memory>
#include <optional>
#include <vector>

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

  void analyze(const DeviceSparseSpdSystem& system,
               CudaDeviceArray<double>* solution,
               cudaStream_t stream = nullptr);
  void analyze(const DeviceSparseSpdSystem& system,
               CudaDeviceArray<double>* solution,
               const std::vector<int>& scalarPermutation,
               cudaStream_t stream = nullptr);
  /**
   * Numerically factor and solve the analyzed SPD system.
   *
   * Throws std::runtime_error when cuDSS reports a non-positive minor during
   * numerical factorization.
   */
  void solve(const DeviceSparseSpdSystem& system,
             CudaDeviceArray<double>* solution, cudaStream_t stream = nullptr);
  void solve(const DeviceSparseSpdSystem& system,
             CudaDeviceArray<double>* solution, cudaStream_t stream,
             CudssSpdSolveProfile* profile);

  const CudaLinearSolveStats& stats() const;
  const std::vector<int>& appliedPermutation() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class GTSAM_EXPORT CudssLinearSolver {
 public:
  void solveSpd(const DeviceSparseSpdSystem& system,
                CudaDeviceArray<double>* solution,
                cudaStream_t stream = nullptr) const;
};

}  // namespace gtsam::cuda
