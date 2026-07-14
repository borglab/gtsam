#pragma once

#include <gtsam/base/cuda/CudaPinnedHostArray.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <cstddef>

namespace gtsam::cuda {

class HostSparseJacobian {
 public:
  explicit HostSparseJacobian(const SparseJacobianPlan& plan)
      : values_(static_cast<size_t>(plan.nonzeros())),
        rhs_(static_cast<size_t>(plan.rows())) {
    clear();
  }

  void clear() {
    values_.clear();
    rhs_.clear();
  }

  double* valuesData() { return values_.data(); }
  const double* valuesData() const { return values_.data(); }
  double* rhsData() { return rhs_.data(); }
  const double* rhsData() const { return rhs_.data(); }
  size_t valuesSize() const { return values_.size(); }
  size_t rhsSize() const { return rhs_.size(); }

 private:
  CudaPinnedHostArray<double> values_;
  CudaPinnedHostArray<double> rhs_;
};

}  // namespace gtsam::cuda
