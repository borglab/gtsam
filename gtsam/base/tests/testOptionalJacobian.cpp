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

  OptionalJacobian<-1, -1> H7;
  EXPECT(!H7);

  OptionalJacobian<-1, -1> H8(dynamic);
  EXPECT(H8);

  OptionalJacobian<-1, -1> H9(boost::none);
  EXPECT(!H9);

  OptionalJacobian<-1, -1> H10(optional);
  EXPECT(H10);
}

//******************************************************************************
Matrix kTestMatrix = (Matrix23() << 11,12,13,21,22,23).finished();

void test(OptionalJacobian<2, 3> H = boost::none) {
  if (H)
    *H = kTestMatrix;
}

TEST( OptionalJacobian, Fixed) {

  // Default argument does nothing
  test();

  // Fixed size, no copy
  Matrix23 fixed1;
  fixed1.setOnes();
  test(fixed1);
  EXPECT(assert_equal(kTestMatrix,fixed1));

  // Fixed size, no copy, pointer style
  Matrix23 fixed2;
  fixed2.setOnes();
  test(&fixed2);
  EXPECT(assert_equal(kTestMatrix,fixed2));

  // Passing in an empty matrix means we want it resized
  Matrix dynamic0;
  test(dynamic0);
  EXPECT(assert_equal(kTestMatrix,dynamic0));

  // Dynamic wrong size
  Matrix dynamic1(3, 5);
  dynamic1.setOnes();
  test(dynamic1);
  EXPECT(assert_equal(kTestMatrix,dynamic1));

  // Dynamic right size
  Matrix dynamic2(2, 5);
  dynamic2.setOnes();
  test(dynamic2);
  EXPECT(assert_equal(kTestMatrix, dynamic2));
}

//******************************************************************************
void test2(OptionalJacobian<-1,-1> H = boost::none) {
  if (H)
    *H = kTestMatrix; // resizes
}

TEST( OptionalJacobian, Dynamic) {

  // Default argument does nothing
  test2();

  // Passing in an empty matrix means we want it resized
  Matrix dynamic0;
  test2(dynamic0);
  EXPECT(assert_equal(kTestMatrix,dynamic0));

  // Dynamic wrong size
  Matrix dynamic1(3, 5);
  dynamic1.setOnes();
  test2(dynamic1);
  EXPECT(assert_equal(kTestMatrix,dynamic1));

  // Dynamic right size
  Matrix dynamic2(2, 5);
  dynamic2.setOnes();
  test2(dynamic2);
  EXPECT(assert_equal(kTestMatrix, dynamic2));
}

//******************************************************************************
void test3(double add, OptionalJacobian<2,1> H = boost::none) {
  if (H) *H << add + 10, add + 20;
}

// This function calls the above function three times, one for each column
void test4(OptionalJacobian<2, 3> H = boost::none) {
  if (H) {
    test3(1, H.cols<1>(0));
    test3(2, H.cols<1>(1));
    test3(3, H.cols<1>(2));
  }
}

TEST(OptionalJacobian, Block) {
  // Default argument does nothing
  test4();

  Matrix23 fixed;
  fixed.setOnes();
  test4(fixed);
  EXPECT(assert_equal(kTestMatrix, fixed));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
