/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testUtilities.cpp
 * @date Aug 19, 2021
 * @author Varun Agrawal
 * @brief Tests for the utilities.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/utilities.h>

using namespace gtsam;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::R;
using gtsam::symbol_shorthand::X;

/* ************************************************************************* */
TEST(Utilities, ExtractPoint2) {
  Point2 p0(0, 0), p1(1, 0);
  Values values;
  values.insert<Point2>(L(0), p0);
  values.insert<Point2>(L(1), p1);
  values.insert<Rot3>(R(0), Rot3());
  values.insert<Pose3>(X(0), Pose3());

  Matrix all_points = utilities::extractPoint2(values);
  EXPECT_LONGS_EQUAL(2, all_points.rows());
}

/* ************************************************************************* */
TEST(Utilities, ExtractPoint3) {
  Point3 p0(0, 0, 0), p1(1, 0, 0);
  Values values;
  values.insert<Point3>(L(0), p0);
  values.insert<Point3>(L(1), p1);
  values.insert<Rot3>(R(0), Rot3());
  values.insert<Pose3>(X(0), Pose3());

  Matrix all_points = utilities::extractPoint3(values);
  EXPECT_LONGS_EQUAL(2, all_points.rows());
}

/* ************************************************************************* */
TEST(Utilities, ExtractVector) {
  // Test normal case with 3 vectors and 1 non-vector (ignore non-vector)
  auto values = Values();
  values.insert(X(0), (Vector(4) << 1, 2, 3, 4).finished());
  values.insert(X(2), (Vector(4) << 13, 14, 15, 16).finished());
  values.insert(X(1), (Vector(4) << 6, 7, 8, 9).finished());
  values.insert(X(3), Pose3());
  auto actual = utilities::extractVectors(values, 'x');
  auto expected =
      (Matrix(3, 4) << 1, 2, 3, 4, 6, 7, 8, 9, 13, 14, 15, 16).finished();
  EXPECT(assert_equal(expected, actual));

  // Check that mis-sized vectors fail
  values.insert(X(4), (Vector(2) << 1, 2).finished());
  THROWS_EXCEPTION(utilities::extractVectors(values, 'x'));
  values.update(X(4), (Vector(6) << 1, 2, 3, 4, 5, 6).finished());
  THROWS_EXCEPTION(utilities::extractVectors(values, 'x'));
}

/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
