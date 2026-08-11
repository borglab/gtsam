/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testStereoPoint2.cpp
 * @brief Tests for the StereoPoint2 class
 *
 * @date Nov 4, 2011
 * @author Alex Cunningham
 */

#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/lieProxies.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(StereoPoint2)
//GTSAM_CONCEPT_MATRIX_LIE_GROUP_INST(StereoPoint2)


//******************************************************************************
TEST(StereoPoint2 , Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<StereoPoint2>);
  GTSAM_CONCEPT_ASSERT(IsManifold<StereoPoint2 >);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<StereoPoint2>);
}

/* ************************************************************************* */
TEST(StereoPoint2, constructor) {
  StereoPoint2 p1(1, 2, 3), p2 = p1;
  EXPECT(assert_equal(p1, p2));
}

/* ************************************************************************* */
TEST( StereoPoint2, arithmetic) {
  EXPECT(assert_equal( StereoPoint2(5,6,7), StereoPoint2(4,5,6)+StereoPoint2(1,1,1)));
  EXPECT(assert_equal( StereoPoint2(3,4,5), StereoPoint2(4,5,6)-StereoPoint2(1,1,1)));
}

/* ************************************************************************* */
TEST(testStereoPoint2, left_right) {
  StereoPoint2 p1(1,2,3);

  EXPECT(assert_equal( Point2(1,3), p1.point2()));
  EXPECT(assert_equal( Point2(2,3), p1.right()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
