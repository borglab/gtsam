#pragma once

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

/// Triangle stored by a symmetric sparse system.
enum class SparseTriangle { Upper, Lower };

/// Prepared-system representation accepted by the shared solver layer.
enum class LinearSystemKind { Dense, Sparse, Operator };

/// Non-owning view of a column-major dense SPD matrix and right-hand side.
struct DenseSpdSystemView {
  int dimension = 0;
  int leadingDimension = 0;
  double* values = nullptr;
  double* rhs = nullptr;
};

/// Non-owning view of a symmetric CSR SPD matrix and right-hand side.
struct SparseSpdSystemView {
  int dimension = 0;
  int nonzeros = 0;
  const int* rowPointers = nullptr;
  const int* columnIndices = nullptr;
  double* values = nullptr;
  double* rhs = nullptr;
  SparseTriangle triangle = SparseTriangle::Upper;
};

/// Matrix-free linear map whose storage remains owned by its frontend.
class LinearOperator {
 public:
  virtual ~LinearOperator() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double* input, double* output,
                     cudaStream_t stream) const = 0;
};

/// Matrix-free inverse approximation paired with a linear operator.
class Preconditioner {
 public:
  virtual ~Preconditioner() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double* input, double* output,
                     cudaStream_t stream) const = 0;
};

}  // namespace gtsam::cuda
