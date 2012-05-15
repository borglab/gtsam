/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testPermutation.cpp
 * @date Sep 22, 2011
 * @author richard
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Permutation.h>

#include <vector>

using namespace gtsam;
using namespace std;

TEST(Permutation, Identity) {
  Permutation expected(5);
  expected[0] = 0;
  expected[1] = 1;
  expected[2] = 2;
  expected[3] = 3;
  expected[4] = 4;

  Permutation actual = Permutation::Identity(5);

  EXPECT(assert_equal(expected, actual));
}

TEST(Permutation, PullToFront) {
  Permutation expected(5);
  expected[0] = 4;
  expected[1] = 0;
  expected[2] = 2;
  expected[3] = 1;
  expected[4] = 3;

  std::vector<Index> toFront;
  toFront.push_back(4);
  toFront.push_back(0);
  toFront.push_back(2);
  Permutation actual = Permutation::PullToFront(toFront, 5);

  EXPECT(assert_equal(expected, actual));
}

TEST(Permutation, PushToBack) {
  Permutation expected(5);
  expected[0] = 1;
  expected[1] = 3;
  expected[2] = 4;
  expected[3] = 0;
  expected[4] = 2;

  std::vector<Index> toBack;
  toBack.push_back(4);
  toBack.push_back(0);
  toBack.push_back(2);
  Permutation actual = Permutation::PushToBack(toBack, 5);

  EXPECT(assert_equal(expected, actual));
}

TEST(Permutation, compose) {
  Permutation p1(5);
  p1[0] = 1;
  p1[1] = 2;
  p1[2] = 3;
  p1[3] = 4;
  p1[4] = 0;

  Permutation p2(5);
  p2[0] = 1;
  p2[1] = 2;
  p2[2] = 4;
  p2[3] = 3;
  p2[4] = 0;

  Permutation expected(5);
  expected[0] = 2;
  expected[1] = 3;
  expected[2] = 0;
  expected[3] = 4;
  expected[4] = 1;

  Permutation actual = *p1.permute(p2);

  EXPECT(assert_equal(expected, actual));
  LONGS_EQUAL(p1[p2[0]], actual[0]);
  LONGS_EQUAL(p1[p2[1]], actual[1]);
  LONGS_EQUAL(p1[p2[2]], actual[2]);
  LONGS_EQUAL(p1[p2[3]], actual[3]);
  LONGS_EQUAL(p1[p2[4]], actual[4]);
}

/* ************************************************************************* */
int main() {  TestResult tr;  return TestRegistry::runAllTests(tr); }


