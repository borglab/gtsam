/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testDCMarginals.cpp
 *  @brief Unit tests for DCMarginals class
 *  @author Varun Agrawal
 *  @date December 2021
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/hybrid/DCMarginals.h>

using namespace std;
using namespace gtsam;
using namespace serializationTestHelpers;

//******************************************************************************
TEST(DCMarginals, Constructor) {
  Marginals continuous;
  DiscreteMarginals discrete;

  DCMarginals dc_marginals(continuous, discrete);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
