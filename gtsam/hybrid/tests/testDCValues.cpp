/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testDCValues.cpp
 *  @brief Unit tests for DCValues class
 *  @author Varun Agrawal
 *  @date December 2021
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/hybrid/DCValues.h>

using namespace std;
using namespace gtsam;
using namespace serializationTestHelpers;

//******************************************************************************
TEST(DCValues, Constructor) { 
  Values contValues;
  DiscreteValues discValues;

  DCValues dc_values(contValues, discValues);
  EXPECT(dc_values.continuous_.size() == 0);
  EXPECT(dc_values.discrete_.size() == 0);
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
