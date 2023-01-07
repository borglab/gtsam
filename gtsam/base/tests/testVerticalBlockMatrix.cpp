/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testVerticalBlockMatrix.cpp
 * @brief  Unit tests for VerticalBlockMatrix class
 * @author Frank Dellaert
 * @date   February 15, 2014
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/VerticalBlockMatrix.h>

#include<list>
#include<vector>

using namespace gtsam;

const std::list<size_t> L{3, 2, 1};
const std::vector<size_t> dimensions(L.begin(), L.end());

//*****************************************************************************
TEST(VerticalBlockMatrix, Constructor1) {
  VerticalBlockMatrix actual(dimensions,6);
  EXPECT_LONGS_EQUAL(6,actual.rows());
  EXPECT_LONGS_EQUAL(6,actual.cols());
  EXPECT_LONGS_EQUAL(3,actual.nBlocks());
}

//*****************************************************************************
TEST(VerticalBlockMatrix, Constructor2) {
  VerticalBlockMatrix actual(dimensions,
      (Matrix(6, 6) << 1, 2, 3, 4, 5, 6, //
      2, 8, 9, 10, 11, 12, //
      3, 9, 15, 16, 17, 18, //
      4, 10, 16, 22, 23, 24, //
      5, 11, 17, 23, 29, 30, //
      6, 12, 18, 24, 30, 36).finished());
  EXPECT_LONGS_EQUAL(6,actual.rows());
  EXPECT_LONGS_EQUAL(6,actual.cols());
  EXPECT_LONGS_EQUAL(3,actual.nBlocks());
}

//*****************************************************************************
TEST(VerticalBlockMatrix, Constructor3) {
  VerticalBlockMatrix actual(dimensions.begin(),dimensions.end(),6);
  EXPECT_LONGS_EQUAL(6,actual.rows());
  EXPECT_LONGS_EQUAL(6,actual.cols());
  EXPECT_LONGS_EQUAL(3,actual.nBlocks());
}

//*****************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************

