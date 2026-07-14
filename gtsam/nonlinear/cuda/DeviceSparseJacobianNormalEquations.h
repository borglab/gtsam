#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>
#include <gtsam/nonlinear/cuda/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <cuda_runtime_api.h>

#include <memory>
#include <string>

namespace gtsam::cuda {

struct DeviceSparseNormalEquationCapability {
  bool supported = false;
  std::string detail;
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
  // until the fixed stream reaches the queued copies.
  void uploadNumerics(const HostSparseJacobian& host,
                      cudaStream_t stream = nullptr);
  void formUndampedSystem(cudaStream_t stream = nullptr);

  const DeviceSparseNormalEquations& system() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
