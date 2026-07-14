#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>
#include <gtsam/nonlinear/cuda/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <memory>
#include <string>

namespace gtsam::cuda {

struct DeviceSparseNormalEquationCapability {
  bool supported = false;
  std::string detail;
};

struct LinearizedModelErrors {
  double oldError = 0.0;
  double newError = 0.0;

  double change() const { return oldError - newError; }
};

struct DeviceSparseJacobianAttemptResult {
  Vector delta;
  LinearizedModelErrors model;
};

class GTSAM_EXPORT DeviceSparseJacobianNormalEquations {
 public:
  DeviceSparseJacobianNormalEquations();
  ~DeviceSparseJacobianNormalEquations();

  DeviceSparseJacobianNormalEquations(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations& operator=(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations(
      DeviceSparseJacobianNormalEquations&&) noexcept;
  DeviceSparseJacobianNormalEquations& operator=(
      DeviceSparseJacobianNormalEquations&&) noexcept;

  static DeviceSparseNormalEquationCapability preflightCapability();

  // The borrowed fixed stream must outlive this object; destruction waits for
  // it before releasing descriptors, workspaces, and device allocations.
  void initialize(const SparseJacobianPlan& plan,
                  cudaStream_t stream = nullptr);
  // This upload is asynchronous. The pinned host storage must remain alive
  // and unmodified until the fixed stream reaches the queued copies.
  void uploadNumerics(const HostSparseJacobian& host,
                      cudaStream_t stream = nullptr);
  void formUndampedSystem(cudaStream_t stream = nullptr);

  // Every stream-taking operation after initialize() must receive the fixed
  // stream. Consumers of system() must use that stream, or establish ordering
  // with an event, before reading or modifying its storage.
  // formUndampedSystem(), prepareDamping(), analyze(), and solveAndEvaluate()
  // are asynchronous except for cuDSS's required numerical-factorization
  // status boundary.
  // downloadAttemptResult() performs the one synchronization needed to return
  // host-owned values.
  void prepareDamping(bool diagonalDamping, double minDiagonal,
                      double maxDiagonal, cudaStream_t stream = nullptr);
  void analyze(cudaStream_t stream = nullptr);
  void solveAndEvaluate(double lambda, cudaStream_t stream = nullptr);
  size_t analysisCount() const;
  DeviceSparseJacobianAttemptResult downloadAttemptResult(
      cudaStream_t stream = nullptr) const;

  const DeviceSparseNormalEquations& system() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
