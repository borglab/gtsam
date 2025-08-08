/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationGeometry.cpp
 * @brief
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Unit3.h>

using namespace std;
using namespace gtsam;
using namespace serializationTestHelpers;

// Add this macro after the static declarations
#define TEST_SERIALIZATION_FORMATS(obj) \
  EXPECT(equalsObj(obj));               \
  EXPECT(equalsXML(obj));               \
  EXPECT(equalsBinary(obj));

/* ************************************************************************* */
static Point3 pt3(1.0, 2.0, 3.0);
static Rot3 rt3 = Rot3::RzRyRx(1.0, 3.0, 2.0);
static Pose3 pose3(rt3, pt3);

static Unit3 unit3(1.0, 2.1, 3.4);
static EssentialMatrix ematrix(rt3, unit3);

static Cal3_S2 cal1(1.0, 2.0, 0.3, 0.1, 0.5);
static Cal3DS2 cal2(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
static Cal3Bundler cal3(1.0, 2.0, 3.0);
static Cal3_S2Stereo cal4(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
static Cal3_S2Stereo::shared_ptr cal4ptr(new Cal3_S2Stereo(cal4));
static CalibratedCamera cal5(Pose3(rt3, pt3));
static Cal3Unified cal6(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0);

static PinholeCamera<Cal3_S2> cam1(pose3, cal1);
static StereoCamera cam2(pose3, cal4ptr);
static StereoPoint2 spt(1.0, 2.0, 3.0);

static const Vector15 xi_sl4 =
    (Vector15() << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10,
     0.11, 0.12, 0.13, 0.14, 0.15)
        .finished();

/* ************************************************************************* */
TEST(Serialization, all_geometry_formats) {
  // Simple types
  TEST_SERIALIZATION_FORMATS(Point2(1.0, 2.0));
  TEST_SERIALIZATION_FORMATS(Pose2(1.0, 2.0, 0.3));
  TEST_SERIALIZATION_FORMATS(Rot2::fromDegrees(30.0));
  TEST_SERIALIZATION_FORMATS(Unit3(1.0, 2.1, 3.4));
  TEST_SERIALIZATION_FORMATS(EssentialMatrix(rt3, unit3));

  // Static objects
  TEST_SERIALIZATION_FORMATS(pt3);
  TEST_SERIALIZATION_FORMATS(rt3);
  TEST_SERIALIZATION_FORMATS(pose3);
  TEST_SERIALIZATION_FORMATS(cal1);
  TEST_SERIALIZATION_FORMATS(cal2);
  TEST_SERIALIZATION_FORMATS(cal3);
  TEST_SERIALIZATION_FORMATS(cal4);
  TEST_SERIALIZATION_FORMATS(cal5);
  TEST_SERIALIZATION_FORMATS(cal6);
  TEST_SERIALIZATION_FORMATS(cam1);
  TEST_SERIALIZATION_FORMATS(cam2);
  TEST_SERIALIZATION_FORMATS(spt);

  TEST_SERIALIZATION_FORMATS(SL4::Expmap(xi_sl4));
  TEST_SERIALIZATION_FORMATS(Similarity3(rt3, pt3, 2.0));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
