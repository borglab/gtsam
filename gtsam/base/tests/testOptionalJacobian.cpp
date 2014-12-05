/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testOptionalJacobian.cpp
 * @brief   Unit test for OptionalJacobian
 * @author  Frank Dellaert
 * @date    Nov 28, 2014
 **/

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//******************************************************************************
TEST( OptionalJacobian, Constructors ) {
  Matrix23 fixed;

  OptionalJacobian<2, 3> H1;
  EXPECT(!H1);

  OptionalJacobian<2, 3> H2(fixed);
  EXPECT(H2);

  OptionalJacobian<2, 3> H3(&fixed);
  EXPECT(H3);

  Matrix dynamic;
  OptionalJacobian<2, 3> H4(dynamic);
  EXPECT(H4);

  OptionalJacobian<2, 3> H5(boost::none);
  EXPECT(!H5);

  boost::optional<Matrix&> optional(dynamic);
  OptionalJacobian<2, 3> H6(optional);
  EXPECT(H6);
}

//******************************************************************************
void test(OptionalJacobian<2, 3> H = boost::none) {
  if (H)
    *H = Matrix23::Zero();
}

void testPtr(Matrix23* H = NULL) {
  if (H)
    *H = Matrix23::Zero();
}

TEST( OptionalJacobian, Ref2) {

  Matrix expected;
  expected = Matrix23::Zero();

  // Default argument does nothing
  test();

  // Fixed size, no copy
  Matrix23 fixed1;
  fixed1.setOnes();
  test(fixed1);
  EXPECT(assert_equal(expected,fixed1));

  // Fixed size, no copy, pointer style
  Matrix23 fixed2;
  fixed2.setOnes();
  test(&fixed2);
  EXPECT(assert_equal(expected,fixed2));

  // Empty is no longer a sign we don't want a matrix, we want it resized
  Matrix dynamic0;
  test(dynamic0);
  EXPECT(assert_equal(expected,dynamic0));

  // Dynamic wrong size
  Matrix dynamic1(3, 5);
  dynamic1.setOnes();
  test(dynamic1);
  EXPECT(assert_equal(expected,dynamic0));

  // Dynamic right size
  Matrix dynamic2(2, 5);
  dynamic2.setOnes();
  test(dynamic2);
  EXPECT(assert_equal(dynamic2,dynamic0));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
