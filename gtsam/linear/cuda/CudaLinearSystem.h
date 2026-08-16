#pragma once

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

/** Triangle stored by a symmetric sparse system. */
enum class CudaSparseTriangle { Upper, Lower };

/** Prepared-system representation accepted by the shared solver layer. */
enum class CudaLinearSystemKind { Dense, Sparse, Operator };

/** Borrowed column-major dense SPD matrix and right-hand side. */
struct CudaDenseSpdSystemView {
  int dimension = 0;
  int leadingDimension = 0;
  double* values = nullptr;
  double* rhs = nullptr;
};

/** Borrowed symmetric CSR SPD matrix and right-hand side. */
struct CudaSparseSpdSystemView {
  int dimension = 0;
  int nonzeros = 0;
  const int* rowPointers = nullptr;
  const int* columnIndices = nullptr;
  double* values = nullptr;
  double* rhs = nullptr;
  CudaSparseTriangle triangle = CudaSparseTriangle::Upper;
};

/** Matrix-free linear map whose storage remains owned by its frontend. */
class CudaLinearOperator {
 public:
  virtual ~CudaLinearOperator() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double* input, double* output,
                     cudaStream_t stream) const = 0;
};

/** Matrix-free inverse approximation paired with a linear operator. */
class CudaPreconditioner {
 public:
  virtual ~CudaPreconditioner() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double* input, double* output,
                     cudaStream_t stream) const = 0;
};

}  // namespace gtsam::cuda
