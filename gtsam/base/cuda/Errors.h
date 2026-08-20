/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Errors.h
 * @brief   Exception type and status checking for CUDA API calls
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

// The kernels accumulate into double-precision buffers with atomicAdd, which
// exists only from compute capability 6.0 (Pascal) onward. Report that here
// rather than letting each kernel fail with "no instance of overloaded
// function atomicAdd matches the argument list". See cmake/HandleCuda.cmake,
// which pins CMAKE_CUDA_ARCHITECTURES so this cannot trigger via our build.
#if defined(__CUDA_ARCH__) && __CUDA_ARCH__ < 600
#error "GTSAM CUDA support requires compute capability 6.0 or newer; set CMAKE_CUDA_ARCHITECTURES accordingly (for example -DCMAKE_CUDA_ARCHITECTURES=native)"
#endif

#include <cuda_runtime_api.h>

#include <sstream>
#include <stdexcept>
#include <string>

namespace gtsam::cuda {

/// Thrown for a failed CUDA runtime call, so these can be caught apart from the
/// std::invalid_argument and std::logic_error the solvers also throw.
class Error : public std::runtime_error {
 public:
  /// Constructs from a message that already identifies the failing call.
  explicit Error(const std::string& message) : std::runtime_error(message) {}
};

/// Throws Error unless status is cudaSuccess, quoting the call and its location.
/// Call through GTSAM_CUDA_CHECK, which fills in the last three arguments.
inline void checkRuntime(cudaError_t status, const char* expression,
                      const char* file, int line) {
  if (status == cudaSuccess) return;
  std::ostringstream os;
  os << "CUDA call failed at " << file << ":" << line << ": " << expression
     << " returned " << cudaGetErrorString(status);
  throw Error(os.str());
}

}  // namespace gtsam::cuda

#define GTSAM_CUDA_CHECK(expr) \
  ::gtsam::cuda::checkRuntime((expr), #expr, __FILE__, __LINE__)
