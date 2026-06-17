#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>

namespace gtsam::cuda {

class CudssLinearSolver {
 public:
  void solveSpd(const DeviceSparseNormalEquations& system,
                CudaDeviceArray<double>* solution,
                cudaStream_t stream = nullptr) const;
};

}  // namespace gtsam::cuda
