/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TestSfmData.cpp
 * @date January 2022
 * @author Frank dellaert
 * @brief tests for SfmData class and associated utilites
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/sfm/SfmData.h>

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::P;
using gtsam::symbol_shorthand::X;

namespace gtsam {
GTSAM_EXPORT std::string createRewrittenFileName(const std::string& name);
GTSAM_EXPORT std::string findExampleDataFile(const std::string& name);
}  // namespace gtsam

/* ************************************************************************* */
TEST(dataSet, Balbianello) {
  ///< The structure where we will save the SfM data
  const string filename = findExampleDataFile("Balbianello");
  SfmData mydata;
  CHECK(readBundler(filename, mydata));

  // Check number of things
  EXPECT_LONGS_EQUAL(5, mydata.nrCameras());
  EXPECT_LONGS_EQUAL(544, mydata.nrTracks());
  const SfmTrack& track0 = mydata.tracks[0];
  EXPECT_LONGS_EQUAL(3, track0.nrMeasurements());

  // Check projection of a given point
  EXPECT_LONGS_EQUAL(0, track0.measurements[0].first);
  const SfmCamera& camera0 = mydata.cameras[0];
  Point2 expected = camera0.project(track0.p),
         actual = track0.measurements[0].second;
  EXPECT(assert_equal(expected, actual, 1));
}

/* ************************************************************************* */
TEST(dataSet, readBAL_Dubrovnik) {
  ///< The structure where we will save the SfM data
  const string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData mydata;
  CHECK(readBAL(filename, mydata));

  // Check number of things
  EXPECT_LONGS_EQUAL(3, mydata.nrCameras());
  EXPECT_LONGS_EQUAL(7, mydata.nrTracks());
  const SfmTrack& track0 = mydata.tracks[0];
  EXPECT_LONGS_EQUAL(3, track0.nrMeasurements());

  // Check projection of a given point
  EXPECT_LONGS_EQUAL(0, track0.measurements[0].first);
  const SfmCamera& camera0 = mydata.cameras[0];
  Point2 expected = camera0.project(track0.p),
         actual = track0.measurements[0].second;
  EXPECT(assert_equal(expected, actual, 12));
}

/* ************************************************************************* */
TEST(dataSet, openGL2gtsam) {
  Vector3 rotVec(0.2, 0.7, 1.1);
  Rot3 R = Rot3::Expmap(rotVec);
  Point3 t = Point3(0.0, 0.0, 0.0);
  Pose3 poseGTSAM = Pose3(R, t);

  Pose3 expected = openGL2gtsam(R, t.x(), t.y(), t.z());

  Point3 r1 = R.r1(), r2 = R.r2(), r3 = R.r3();  // columns!
  Rot3 cRw(r1.x(), r2.x(), r3.x(), -r1.y(), -r2.y(), -r3.y(), -r1.z(), -r2.z(),
           -r3.z());
  Rot3 wRc = cRw.inverse();
  Pose3 actual = Pose3(wRc, t);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(dataSet, gtsam2openGL) {
  Vector3 rotVec(0.2, 0.7, 1.1);
  Rot3 R = Rot3::Expmap(rotVec);
  Point3 t = Point3(1.0, 20.0, 10.0);
  Pose3 actual = Pose3(R, t);
  Pose3 poseGTSAM = openGL2gtsam(R, t.x(), t.y(), t.z());

  Pose3 expected = gtsam2openGL(poseGTSAM);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(dataSet, writeBAL_Dubrovnik) {
  ///< Read a file using the unit tested readBAL
  const string filenameToRead = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData readData;
  readBAL(filenameToRead, readData);

  // Write readData to file filenameToWrite
  const string filenameToWrite = createRewrittenFileName(filenameToRead);
  CHECK(writeBAL(filenameToWrite, readData));

  // Read what we wrote
  SfmData writtenData;
  CHECK(readBAL(filenameToWrite, writtenData));

  // Check that what we read is the same as what we wrote
  EXPECT_LONGS_EQUAL(readData.nrCameras(), writtenData.nrCameras());
  EXPECT_LONGS_EQUAL(readData.nrTracks(), writtenData.nrTracks());

  for (size_t i = 0; i < readData.nrCameras(); i++) {
    PinholeCamera<Cal3Bundler> expectedCamera = writtenData.cameras[i];
    PinholeCamera<Cal3Bundler> actualCamera = readData.cameras[i];
    EXPECT(assert_equal(expectedCamera, actualCamera));
  }

  for (size_t j = 0; j < readData.nrTracks(); j++) {
    // check point
    SfmTrack expectedTrack = writtenData.tracks[j];
    SfmTrack actualTrack = readData.tracks[j];
    Point3 expectedPoint = expectedTrack.p;
    Point3 actualPoint = actualTrack.p;
    EXPECT(assert_equal(expectedPoint, actualPoint));

    // check rgb
    Point3 expectedRGB =
        Point3(expectedTrack.r, expectedTrack.g, expectedTrack.b);
    Point3 actualRGB = Point3(actualTrack.r, actualTrack.g, actualTrack.b);
    EXPECT(assert_equal(expectedRGB, actualRGB));

    // check measurements
    for (size_t k = 0; k < actualTrack.nrMeasurements(); k++) {
      EXPECT_LONGS_EQUAL(expectedTrack.measurements[k].first,
                         actualTrack.measurements[k].first);
      EXPECT(assert_equal(expectedTrack.measurements[k].second,
                          actualTrack.measurements[k].second));
    }
  }
}

/* ************************************************************************* */
TEST(dataSet, writeBALfromValues_Dubrovnik) {
  ///< Read a file using the unit tested readBAL
  const string filenameToRead = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData readData;
  readBAL(filenameToRead, readData);

  Pose3 poseChange =
      Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10), Point3(0.3, 0.1, 0.3));

  Values value;
  for (size_t i = 0; i < readData.nrCameras(); i++) {  // for each camera
    Pose3 pose = poseChange.compose(readData.cameras[i].pose());
    value.insert(X(i), pose);
  }
  for (size_t j = 0; j < readData.nrTracks(); j++) {  // for each point
    Point3 point = poseChange.transformFrom(readData.tracks[j].p);
    value.insert(P(j), point);
  }

  // Write values and readData to a file
  const string filenameToWrite = createRewrittenFileName(filenameToRead);
  writeBALfromValues(filenameToWrite, readData, value);

  // Read the file we wrote
  SfmData writtenData;
  readBAL(filenameToWrite, writtenData);

  // Check that the reprojection errors are the same and the poses are correct
  // Check number of things
  EXPECT_LONGS_EQUAL(3, writtenData.nrCameras());
  EXPECT_LONGS_EQUAL(7, writtenData.nrTracks());
  const SfmTrack& track0 = writtenData.tracks[0];
  EXPECT_LONGS_EQUAL(3, track0.nrMeasurements());

  // Check projection of a given point
  EXPECT_LONGS_EQUAL(0, track0.measurements[0].first);
  const SfmCamera& camera0 = writtenData.cameras[0];
  Point2 expected = camera0.project(track0.p),
         actual = track0.measurements[0].second;
  EXPECT(assert_equal(expected, actual, 12));

  Pose3 expectedPose = camera0.pose();
  Pose3 actualPose = value.at<Pose3>(X(0));
  EXPECT(assert_equal(expectedPose, actualPose, 1e-7));

  Point3 expectedPoint = track0.p;
  Point3 actualPoint = value.at<Point3>(P(0));
  EXPECT(assert_equal(expectedPoint, actualPoint, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
