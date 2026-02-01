/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TestSfmTrack.cpp
 * @date October 2022
 * @author Frank Dellaert
 * @brief tests for SfmTrack class
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/SfmTrack.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(SfmTrack2d, defaultConstructor) {
  SfmTrack2d track;
  EXPECT_LONGS_EQUAL(0, track.measurements.size());
  EXPECT_LONGS_EQUAL(0, track.siftIndices.size());
}

/* ************************************************************************* */
TEST(SfmTrack2d, measurementConstructor) {
  SfmTrack2d track({{0, Point2(1, 2)}, {1, Point2(3, 4)}});
  EXPECT_LONGS_EQUAL(2, track.measurements.size());
  EXPECT_LONGS_EQUAL(0, track.siftIndices.size());
}

/* ************************************************************************* */
TEST(SfmTrack, construction) {
  SfmTrack track(Point3(1, 2, 3), 4, 5, 6);
  EXPECT(assert_equal(Point3(1, 2, 3), track.point3()));
  EXPECT(assert_equal(Point3(4, 5, 6), track.rgb()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
