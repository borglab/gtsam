/* GTSAM Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#pragma once

#include <cuda_runtime_api.h>
#include <gtsam/dllexport.h>

namespace gtsam::cuda::internal {

/// Save the original dense system diagonal before an in-place factorization.
GTSAM_EXPORT void saveCholeskyDiagonal(const double* matrix, int dimension,
                                       int leadingDimension, double* diagonal,
                                       cudaStream_t stream);

/// Apply the shared CPU/CUDA pivot safeguard, preserving any existing error.
/// On rejection info becomes a positive, one-based pivot index (as in potrf).
GTSAM_EXPORT void checkCholeskyPivots(const double* factor, int dimension,
                                      int leadingDimension,
                                      const double* referenceDiagonal,
                                      int* info, cudaStream_t stream);

}  // namespace gtsam::cuda::internal
