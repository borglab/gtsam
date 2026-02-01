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
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/utilities.h>

using namespace gtsam;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::R;
using gtsam::symbol_shorthand::X;

static constexpr double kTol = 1e-9;

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
TEST(Utilities, PerturbPoint2) {
  Values base;
  base.insert<Point2>(L(0), Point2(1.0, 2.0));
  base.insert<Vector>(L(1), (Vector(2) << 3.0, 4.0).finished());

  Values v1 = base, v2 = base;
  utilities::perturbPoint2(v1, /*sigma=*/0.1, /*seed=*/42u);
  utilities::perturbPoint2(v2, /*sigma=*/0.1, /*seed=*/42u);
  EXPECT(assert_equal(v1, v2, kTol));

  const Point2 p0 = base.at<Point2>(L(0));
  const Point2 p1 = v1.at<Point2>(L(0));
  EXPECT((p1 - p0).norm() > 0.0);

  const Vector w0 = base.at<Vector>(L(1));
  const Vector w1 = v1.at<Vector>(L(1));
  EXPECT((w1 - w0).norm() > 0.0);

  Values v3 = base;
  utilities::perturbPoint2(v3, /*sigma=*/0.0, /*seed=*/42u);
  EXPECT(assert_equal(v3, base, kTol));
}

/* ************************************************************************* */
TEST(Utilities, PerturbPoint3) {
  Values base;
  base.insert<Point3>(L(0), Point3(1.0, 2.0, 3.0));
  base.insert<Vector>(L(1), (Vector(3) << 4.0, 5.0, 6.0).finished());

  Values v1 = base, v2 = base;
  utilities::perturbPoint3(v1, /*sigma=*/0.1, /*seed=*/42u);
  utilities::perturbPoint3(v2, /*sigma=*/0.1, /*seed=*/42u);
  EXPECT(assert_equal(v1, v2, kTol));

  const Point3 p0 = base.at<Point3>(L(0));
  const Point3 p1 = v1.at<Point3>(L(0));
  EXPECT((p1 - p0).norm() > 0.0);

  const Vector w0 = base.at<Vector>(L(1));
  const Vector w1 = v1.at<Vector>(L(1));
  EXPECT((w1 - w0).norm() > 0.0);

  Values v3 = base;
  utilities::perturbPoint3(v3, /*sigma=*/0.0, /*seed=*/42u);
  EXPECT(assert_equal(v3, base, kTol));
}

/* ************************************************************************* */
TEST(Utilities, PerturbPose2) {
  Values base;
  const Pose2 pose0(1.0, 2.0, 0.3);
  base.insert<Pose2>(X(0), pose0);

  Values v1 = base, v2 = base;
  utilities::perturbPose2(v1, /*sigmaT=*/0.1, /*sigmaR=*/0.2, /*seed=*/42u);
  utilities::perturbPose2(v2, /*sigmaT=*/0.1, /*sigmaR=*/0.2, /*seed=*/42u);
  EXPECT(assert_equal(v1, v2, kTol));

  const Pose2 pose1 = v1.at<Pose2>(X(0));
  EXPECT(!pose1.equals(pose0, kTol));

  Values v3 = base;
  utilities::perturbPose2(v3, /*sigmaT=*/0.0, /*sigmaR=*/0.0, /*seed=*/42u);
  EXPECT(assert_equal(v3, base, kTol));
}

/* ************************************************************************* */
TEST(Utilities, PerturbPose3) {
  Values base;
  const Pose3 pose0;
  base.insert<Pose3>(X(0), pose0);

  Values v1 = base, v2 = base;
  utilities::perturbPose3(v1, /*sigmaT=*/0.1, /*sigmaR=*/0.2, /*seed=*/42u);
  utilities::perturbPose3(v2, /*sigmaT=*/0.1, /*sigmaR=*/0.2, /*seed=*/42u);
  EXPECT(assert_equal(v1, v2, kTol));

  const Pose3 pose1 = v1.at<Pose3>(X(0));
  EXPECT(!pose1.equals(pose0, kTol));

  Values v3 = base;
  utilities::perturbPose3(v3, /*sigmaT=*/0.0, /*sigmaR=*/0.0, /*seed=*/42u);
  EXPECT(assert_equal(v3, base, kTol));
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
