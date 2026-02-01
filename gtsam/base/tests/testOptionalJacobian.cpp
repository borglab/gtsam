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

#include <optional>
#include <functional>

using namespace std;
using namespace gtsam;

//******************************************************************************
#define TEST_CONSTRUCTOR(DIM1, DIM2, X, TRUTHY) \
  {                                             \
    OptionalJacobian<DIM1, DIM2> H(X);          \
    EXPECT(H == TRUTHY);                        \
  }
TEST( OptionalJacobian, Constructors ) {
  Matrix23 fixed;
  Matrix dynamic;
  std::optional<std::reference_wrapper<Matrix>> optionalRef(std::ref(dynamic));

  OptionalJacobian<2, 3> H;
  EXPECT(!H);

  TEST_CONSTRUCTOR(2, 3, fixed, true);
  TEST_CONSTRUCTOR(2, 3, &fixed, true);
  TEST_CONSTRUCTOR(2, 3, dynamic, true);
  TEST_CONSTRUCTOR(2, 3, &dynamic, true);
  TEST_CONSTRUCTOR(2, 3, std::nullopt, false);
  TEST_CONSTRUCTOR(2, 3, optionalRef, true);

  // Test dynamic
  OptionalJacobian<-1, -1> H7;
  EXPECT(!H7);

  TEST_CONSTRUCTOR(-1, -1, dynamic, true);
  TEST_CONSTRUCTOR(-1, -1, std::nullopt, false);
  TEST_CONSTRUCTOR(-1, -1, optionalRef, true);

}

//******************************************************************************
Matrix kTestMatrix = (Matrix23() << 11,12,13,21,22,23).finished();

void test(OptionalJacobian<2, 3> H = {}) {
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

  {  // Dynamic pointer
    // Passing in an empty matrix means we want it resized
    Matrix dynamic0;
    test(&dynamic0);
    EXPECT(assert_equal(kTestMatrix, dynamic0));

    // Dynamic wrong size
    Matrix dynamic1(3, 5);
    dynamic1.setOnes();
    test(&dynamic1);
    EXPECT(assert_equal(kTestMatrix, dynamic1));

    // Dynamic right size
    Matrix dynamic2(2, 5);
    dynamic2.setOnes();
    test(&dynamic2);
    EXPECT(assert_equal(kTestMatrix, dynamic2));
  }
}

//******************************************************************************
void test2(OptionalJacobian<-1,-1> H = {}) {
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
void test3(double add, OptionalJacobian<2,1> H = {}) {
  if (H) *H << add + 10, add + 20;
}

// This function calls the above function three times, one for each column
void test4(OptionalJacobian<2, 3> H = {}) {
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
