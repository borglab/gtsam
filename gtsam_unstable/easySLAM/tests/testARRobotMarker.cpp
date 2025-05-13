/**********************************************************
Written by Alireza Fathi, 17th of Nov 2008
It tests the ARRobotMarker class from ARRobotMarker.h
**********************************************************/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/SimpleCamera.h>

#include <fstream>

#include "EasySLAMConfig.h"
#include "easyExample.h"

using namespace std;
using namespace gtsam;

#define DEGREES_PER_RADIAN 57.2958
inline double d2r(double degrees) { return degrees / DEGREES_PER_RADIAN; }
inline double r2d(double rad) { return rad * DEGREES_PER_RADIAN; }
inline Vector r2d(const Vector &v) { return v * DEGREES_PER_RADIAN; }

void printPose3(Pose3 p) {
  cout << "T: ";
  print(p.translation().vector());
  cout << "R: ";
  print(r2d(RQ(p.rotation().matrix())));
}

double error = 1e-9;

/* ************************************************************************* */
TEST(ARRobotMarker, ground_truth_constructor_1) {
  // camera
  Cal3_S2 K(625, 625, 0, 0, 0);
  Pose3 pose(rodriguez(0, 0, 0), Point3(0, 0, -100));
  SimpleCamera camera(K, pose);

  // create the marker
  Pose3 marker(rodriguez(d2r(-180), 0, 0), Point3(0, 0, 900));
  double msize = 160;

  // measurement
  ARRobotMarker test_marker(0, 0, camera, marker, msize);

  // test
  Pose3 actual = test_marker.getEstimatedPose();
  Pose3 expected(rodriguez(d2r(-180), 0, 0), Point3(0, 0, 1000));
  CHECK(assert_equal(actual, expected, 0.001));
}

/* ************************************************************************* */
TEST(ARRobotMarker, ground_truth_constructor_2) {
  Pose3 robot(rodriguez(0, 0, 0), Point3(100, 300, -100));
  Pose3 marker(rodriguez(d2r(-180), 0, 0), Point3(100, 300, 900));
  Pose3 expected(rodriguez(d2r(-180), 0, 0), Point3(0, 0, 1000));

  // create the marker
  double msize = 160;
  Cal3_S2 K(625, 625, 0, 0, 0);

  ARRobotMarker test_marker(0, 0, robot, marker, msize, K);

  Pose3 actual = test_marker.getEstimatedPose();

  CHECK(assert_equal(actual, expected, 0.001));
}

/* ************************************************************************* */
TEST(ARRobotMarker, ground_truth_constructor_3) {
  Pose3 robot(Matrix_(3, 3, 1., 0., 0., 0., -1., 0., 0., 0., -1.),
              Point3(0, 0, 1000));
  Pose3 marker(rodriguez(0, 0, 0), Point3(0, 0, 0));

  // create the marker
  double msize = 160;
  Cal3_S2 K(625, 625, 0, 0, 0);
  ARRobotMarker test_marker(0, 0, robot, marker, msize, K);

  const Vector expected =
      Vector_(8, -50.0, 50.0, -50.0, -50.0, 50.0, -50.0, 50.0, 50.0);
  CHECK(assert_equal(test_marker.measurement(), expected));
}

/* ************************************************************************* */
TEST(ARRobotMarker, ground_truth_constructor_4) {
  Pose3 robot(Matrix_(3, 3, 1., 0., 0., 0., -1., 0., 0., 0., -1.),
              Point3(0, 0, 1000));
  Pose3 marker(rodriguez(0, d2r(-45), 0), Point3(0, 0, 0));

  // create the marker
  double msize = 160;
  Cal3_S2 K(625, 625, 0, 0, 0);
  ARRobotMarker test_marker(0, 0, robot, marker, msize, K);

  const Vector expected = Vector_(8, -33.4624, 47.323, -33.4624, -47.323,
                                  37.4753, -52.998, 37.4753, 52.998);
  CHECK(assert_equal(test_marker.measurement(), expected, 0.001));
}

/* ************************************************************************* */
TEST(ARRobotMarker, ground_truth_constructor_5) {
  Pose3 robot(Matrix_(3, 3, 0., 0., 1., 0., -1., 0., 1., 0., 0.),
              Point3(-1000, 0, 0));
  Pose3 marker(rodriguez(0, d2r(-45), 0), Point3(0, 0, 0));

  // create the marker
  double msize = 160;
  Cal3_S2 K(625, 625, 0, 0, 0);
  ARRobotMarker test_marker(0, 0, robot, marker, msize, K);

  const Vector expected = Vector_(8, -37.4752, 52.998, -37.4752, -52.998,
                                  33.4624, -47.323, 33.4624, 47.323);
  CHECK(assert_equal(test_marker.measurement(), expected, 0.001));
}

/* ************************************************************************* */
TEST(ARRobotMarker, measurement) {
  // creating a pair of robot marker
  ARRobotMarker newMarker;
  newMarker.setRobotNumber(1);
  newMarker.setMarkerNumber(1);
  Vector point(2);
  point(0) = -123;
  point(1) = 213456;
  newMarker.setPoint(1, point);
  point(0) = -233;
  point(1) = 215656;
  newMarker.setPoint(2, point);
  point(0) = -563;
  point(1) = 212126;
  newMarker.setPoint(3, point);
  point(0) = -73;
  point(1) = 21312;
  newMarker.setPoint(4, point);

  Vector expected(8);
  expected(0) = -123;
  expected(1) = 213456;
  expected(2) = -233;
  expected(3) = 215656;
  expected(4) = -563;
  expected(5) = 212126;
  expected(6) = -73;
  expected(7) = 21312;

  CHECK(newMarker.measurement() == expected);
}

/* ************************************************************************* */
TEST(ARRobotMarker, estimate_pose2) {
  int NUM_OF_FRAMES = 2;
  string path = "../data/2008_10_01_BorgLab/";
  Cal3_S2 K;
  K.load(path);
  double markerSize = 80.0;
  vector<ARRobotMarker *> markers = loadARToolKit(path, NUM_OF_FRAMES);
  // printf("hello\n");
  ARRobotMarker markerMeasurement = **(markers.begin());
  EasySLAMConfig config(path, NUM_OF_FRAMES);
  // config.print("hello");
  //  actual from homography
  Pose3 cTm = markerMeasurement.getEstimatedPose();

  // config.print();

  // expected from MATLAB
  Pose3 wTc = config.robotPose(0);
  Pose3 wTm;
  Pose3 cTw = wTc.inverse();
  Pose3 cTm2 = cTw * wTm;

  // look at tolerances: why does matlab give different answer ??
  // it also uses the homography !
  CHECK(assert_equal(cTm.translation(), cTm2.translation(), 200.0));
  CHECK(assert_equal(cTm.rotation().matrix(), cTm2.rotation().matrix(), 1.0));
}

/* ************************************************************************* */
TEST(ARRobotMarker, dump_AND_load_dumped) {
  int NUM_OF_FRAMES = 2;
  string path = "../data/EasyExample";
  vector<ARRobotMarker *> markers = loadARToolKit(path, NUM_OF_FRAMES);

  char buffer[100];
  buffer[0] = 0;
  sprintf(buffer, "%s/AR_dump.txt", path.c_str());

  dumpAR(string(buffer), markers);
  vector<ARRobotMarker *> markers2 = load_dumpedAR(string(buffer));

  vector<ARRobotMarker *>::iterator it = markers.begin();
  vector<ARRobotMarker *>::iterator it2 = markers.begin();
  for (; it != markers.end(); it++, it2++) CHECK((*it)->equals(**it2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  TestRegistry::runAllTests(tr);
  return 0;
}
/* ************************************************************************* */
