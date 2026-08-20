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

#include <cuda_runtime_api.h>

#include <sstream>
#include <stdexcept>
#include <string>

namespace gtsam::cuda {

class Error : public std::runtime_error {
 public:
  explicit Error(const std::string& message) : std::runtime_error(message) {}
};

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
