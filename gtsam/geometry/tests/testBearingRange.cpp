/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testBearingRange.cpp
 *  @brief Unit tests for BearingRange Class
 *  @author Frank Dellaert
 *  @date July 2015
 */

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/serializationTestHelpers.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef BearingRange<Pose2, Point2> BearingRange2D;
BearingRange2D br2D(1, 2);

typedef BearingRange<Pose3, Point3> BearingRange3D;
BearingRange3D br3D(Pose3().bearing(Point3(1, 0, 0)), 1);

//******************************************************************************
TEST(BearingRange2D, Concept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<BearingRange2D>);
}

/* ************************************************************************* */
TEST(BearingRange, 2D) {
  BearingRange2D expected(0, 1);
  BearingRange2D actual = BearingRange2D::Measure(Pose2(), Point2(1, 0));
  EXPECT(assert_equal(expected, actual));
}

//******************************************************************************
TEST(BearingRange3D, Concept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<BearingRange3D>);
}

/* ************************************************************************* */
TEST(BearingRange, 3D) {
  BearingRange3D expected(Unit3(), 1);
  BearingRange3D actual = BearingRange3D::Measure(Pose3(), Point3(1, 0, 0));
  EXPECT(assert_equal(expected, actual));
}

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
using namespace serializationTestHelpers;
/* ************************************************************************* */
TEST(BearingRange, Serialization2D) {
  EXPECT(equalsObj<BearingRange2D>(br2D));
  EXPECT(equalsXML<BearingRange2D>(br2D));
  EXPECT(equalsBinary<BearingRange2D>(br2D));
}

/* ************************************************************************* */
TEST(BearingRange, Serialization3D) {
  EXPECT(equalsObj<BearingRange3D>(br3D));
  EXPECT(equalsXML<BearingRange3D>(br3D));
  EXPECT(equalsBinary<BearingRange3D>(br3D));
}
#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
