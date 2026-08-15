#pragma once

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

enum class CudaSparseTriangle { Upper, Lower };

enum class CudaLinearSystemKind { Dense, Sparse, Operator };

struct CudaDenseSpdSystemView {
  int dimension = 0;
  int leadingDimension = 0;
  double* values = nullptr;
  double* rhs = nullptr;
};

struct CudaSparseSpdSystemView {
  int dimension = 0;
  int nonzeros = 0;
  const int* rowPointers = nullptr;
  const int* columnIndices = nullptr;
  double* values = nullptr;
  double* rhs = nullptr;
  CudaSparseTriangle triangle = CudaSparseTriangle::Upper;
};

class CudaLinearOperator {
 public:
  virtual ~CudaLinearOperator() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double* input, double* output,
                     cudaStream_t stream) const = 0;
};

class CudaPreconditioner {
 public:
  virtual ~CudaPreconditioner() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double* input, double* output,
                     cudaStream_t stream) const = 0;
};

}  // namespace gtsam::cuda
