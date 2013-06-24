/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartRangeFactor.cpp
 *  @brief Unit tests for SmartRangeFactor Class
 *  @author Frank Dellaert
 *  @date Nov 2013
 */

#include <gtsam_unstable/slam/SmartRangeFactor.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

const noiseModel::Base::shared_ptr gaussian = noiseModel::Isotropic::Sigma(1,
    2.0);

/* ************************************************************************* */

TEST( SmartRangeFactor, constructor ) {
  SmartRangeFactor f1;
  LONGS_EQUAL(0, f1.nrMeasurements())
  SmartRangeFactor f2(gaussian);
  LONGS_EQUAL(0, f2.nrMeasurements())
}

/* ************************************************************************* */

TEST( SmartRangeFactor, addRange ) {
  SmartRangeFactor f(gaussian);
  f.addRange(1, 10);
  f.addRange(1, 12);
  LONGS_EQUAL(2, f.nrMeasurements())
}

/* ************************************************************************* */

TEST( SmartRangeFactor, unwhitenedError ) {
  // Test situation:
  Point2 p(0, 10);
  Pose2 pose1(0, 0, 0), pose2(5, 0, 0), pose3(5, 5, 0);
  double r1 = pose1.range(p), r2 = pose2.range(p), r3 = pose3.range(p);
  DOUBLES_EQUAL(10, r1, 1e-9);
  DOUBLES_EQUAL(sqrt(100+25), r2, 1e-9);
  DOUBLES_EQUAL(sqrt(50), r3, 1e-9);

  Values values; // all correct
  values.insert(1, pose1);
  values.insert(2, pose2);
  values.insert(3, pose3);

  SmartRangeFactor f(gaussian);
  f.addRange(1, r1);

  // Whenever there are two ranges or less, error should be zero
  Vector actual1 = f.unwhitenedError(values);
  EXPECT(assert_equal(Vector_(1,0.0), actual1));
  f.addRange(2, r2);
  Vector actual2 = f.unwhitenedError(values);
  EXPECT(assert_equal(Vector2(0,0), actual2));

  f.addRange(3, r3);
  vector<Matrix> H;
  Vector actual3 = f.unwhitenedError(values,H);
  EXPECT(assert_equal(Vector3(0,0,0), actual3));

  // Check keys and Jacobian
  EXPECT_LONGS_EQUAL(3,f.keys().size());
  EXPECT(assert_equal(Matrix_(1,3,0.0,-1.0,0.0), H.front()));
  EXPECT(assert_equal(Matrix_(1,3,sqrt(2)/2,-sqrt(2)/2,0.0), H.back()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

