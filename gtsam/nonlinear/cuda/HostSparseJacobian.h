#pragma once

#include <gtsam/base/cuda/PinnedHostArray.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <cstddef>
#include <cstdint>

namespace gtsam::cuda {

class HostSparseJacobian {
 public:
  explicit HostSparseJacobian(const SparseJacobianPlan& plan)
      : structuralFingerprint_(plan.structuralFingerprint()),
        values_(static_cast<size_t>(plan.nonzeros())),
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
  uint64_t structuralFingerprint() const { return structuralFingerprint_; }

 private:
  uint64_t structuralFingerprint_ = 0;
  PinnedHostArray<double> values_;
  PinnedHostArray<double> rhs_;
};

}  // namespace gtsam::cuda
