/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaNames.cpp
 * @brief   Unit tests that CUDA utilities use namespace-aware names
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#include <gtsam/base/cuda/Context.h>
#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/base/cuda/Errors.h>
#include <gtsam/base/cuda/PinnedHostArray.h>

#include <CppUnitLite/TestHarness.h>

#include <type_traits>

using namespace gtsam;

// Verifies that CUDA utility names rely on the enclosing cuda namespace.
TEST(CudaNames, BaseUtilitiesUseNamespaceAwareNames) {
  EXPECT((std::is_default_constructible_v<cuda::Context>));
  EXPECT((std::is_default_constructible_v<cuda::DeviceArray<double>>));
  EXPECT((std::is_default_constructible_v<cuda::PinnedHostArray<double>>));
  EXPECT((std::is_base_of_v<std::runtime_error, cuda::Error>));
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
