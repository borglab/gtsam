#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSystem.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

namespace gtsam::cuda {

/** Matrix-vector view of an explicitly assembled symmetric upper-CSR SPD system. */
class GTSAM_EXPORT SparseSpdOperator final : public LinearOperator {
 public:
  explicit SparseSpdOperator(const DeviceSparseSpdSystem& system)
      : system_(&system) {}
  int dimension() const override { return system_->rows(); }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

 private:
  const DeviceSparseSpdSystem* system_;
};

/** Jacobi inverse for an explicitly assembled upper-CSR SPD system. */
class GTSAM_EXPORT SparseSpdJacobiPreconditioner final
    : public Preconditioner {
 public:
  void build(const DeviceSparseSpdSystem& system,
             cudaStream_t stream = nullptr);
  int dimension() const override { return dimension_; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

 private:
  int dimension_ = 0;
  DeviceArray<double> inverseDiagonal_;
};

}  // namespace gtsam::cuda
