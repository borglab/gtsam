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

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3DS2_k3.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

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
static Cal3DS2_k3 cal7(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0);

static PinholeCamera<Cal3_S2> cam1(pose3, cal1);
static StereoCamera cam2(pose3, cal4ptr);
static StereoPoint2 spt(1.0, 2.0, 3.0);

/* ************************************************************************* */
TEST (Serialization, text_geometry) {
  EXPECT(equalsObj<gtsam::Point2>(Point2(1.0, 2.0)));
  EXPECT(equalsObj<gtsam::Pose2>(Pose2(1.0, 2.0, 0.3)));
  EXPECT(equalsObj<gtsam::Rot2>(Rot2::fromDegrees(30.0)));

  EXPECT(equalsObj<gtsam::Unit3>(Unit3(1.0, 2.1, 3.4)));
  EXPECT(equalsObj<gtsam::EssentialMatrix>(EssentialMatrix(rt3, unit3)));

  EXPECT(equalsObj(pt3));
  EXPECT(equalsObj<gtsam::Rot3>(rt3));
  EXPECT(equalsObj<gtsam::Pose3>(Pose3(rt3, pt3)));

  EXPECT(equalsObj(cal1));
  EXPECT(equalsObj(cal2));
  EXPECT(equalsObj(cal3));
  EXPECT(equalsObj(cal4));
  EXPECT(equalsObj(cal5));
  EXPECT(equalsObj(cal6));
  EXPECT(equalsObj(cal7));

  EXPECT(equalsObj(cam1));
  EXPECT(equalsObj(cam2));
  EXPECT(equalsObj(spt));
}

/* ************************************************************************* */
TEST (Serialization, xml_geometry) {
  EXPECT(equalsXML<gtsam::Point2>(Point2(1.0, 2.0)));
  EXPECT(equalsXML<gtsam::Pose2>(Pose2(1.0, 2.0, 0.3)));
  EXPECT(equalsXML<gtsam::Rot2>(Rot2::fromDegrees(30.0)));

  EXPECT(equalsXML<gtsam::Unit3>(Unit3(1.0, 2.1, 3.4)));
  EXPECT(equalsXML<gtsam::EssentialMatrix>(EssentialMatrix(rt3, unit3)));

  EXPECT(equalsXML<gtsam::Point3>(pt3));
  EXPECT(equalsXML<gtsam::Rot3>(rt3));
  EXPECT(equalsXML<gtsam::Pose3>(Pose3(rt3, pt3)));

  EXPECT(equalsXML(cal1));
  EXPECT(equalsXML(cal2));
  EXPECT(equalsXML(cal3));
  EXPECT(equalsXML(cal4));
  EXPECT(equalsXML(cal5));
  EXPECT(equalsXML(cal7));

  EXPECT(equalsXML(cam1));
  EXPECT(equalsXML(cam2));
  EXPECT(equalsXML(spt));
}

/* ************************************************************************* */
TEST (Serialization, binary_geometry) {
  EXPECT(equalsBinary<gtsam::Point2>(Point2(1.0, 2.0)));
  EXPECT(equalsBinary<gtsam::Pose2>(Pose2(1.0, 2.0, 0.3)));
  EXPECT(equalsBinary<gtsam::Rot2>(Rot2::fromDegrees(30.0)));

  EXPECT(equalsBinary<gtsam::Unit3>(Unit3(1.0, 2.1, 3.4)));
  EXPECT(equalsBinary<gtsam::EssentialMatrix>(EssentialMatrix(rt3, unit3)));

  EXPECT(equalsBinary<gtsam::Point3>(pt3));
  EXPECT(equalsBinary<gtsam::Rot3>(rt3));
  EXPECT(equalsBinary<gtsam::Pose3>(Pose3(rt3, pt3)));

  EXPECT(equalsBinary(cal1));
  EXPECT(equalsBinary(cal2));
  EXPECT(equalsBinary(cal3));
  EXPECT(equalsBinary(cal4));
  EXPECT(equalsBinary(cal5));
  EXPECT(equalsBinary(cal7)); 

  EXPECT(equalsBinary(cam1));
  EXPECT(equalsBinary(cam2));
  EXPECT(equalsBinary(spt));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
