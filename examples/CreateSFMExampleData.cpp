/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CreateSFMExampleData.cpp
 * @brief   Create some example data that for inclusion in the data folder
 * @author  Frank Dellaert
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/CalibratedCamera.h>

#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */

void createExampleBALFile(const string& filename, const vector<Point3>& P,
    const Pose3& pose1, const Pose3& pose2, const Cal3Bundler& K =
        Cal3Bundler()) {

  // Class that will gather all data
  SfM_data data;

  // Create two cameras
  Rot3 aRb = Rot3::Yaw(M_PI_2);
  Point3 aTb(0.1, 0, 0);
  Pose3 identity, aPb(aRb, aTb);
  data.cameras.push_back(SfM_Camera(pose1, K));
  data.cameras.push_back(SfM_Camera(pose2, K));

  for(const Point3& p: P) {

    // Create the track
    SfM_Track track;
    track.p = p;
    track.r = 1;
    track.g = 1;
    track.b = 1;

    // Project points in both cameras
    for (size_t i = 0; i < 2; i++)
    track.measurements.push_back(make_pair(i, data.cameras[i].project(p)));

    // Add track to data
    data.tracks.push_back(track);
  }

  writeBAL(filename, data);
}

/* ************************************************************************* */

void create5PointExample1() {

  // Create two cameras poses
  Rot3 aRb = Rot3::Yaw(M_PI_2);
  Point3 aTb(0.1, 0, 0);
  Pose3 pose1, pose2(aRb, aTb);

  // Create test data, we need at least 5 points
  vector<Point3> P;
  P += Point3(0, 0, 1), Point3(-0.1, 0, 1), Point3(0.1, 0, 1), //
  Point3(0, 0.5, 0.5), Point3(0, -0.5, 0.5);

  // Assumes example is run in ${GTSAM_TOP}/build/examples
  const string filename = "../../examples/data/5pointExample1.txt";
  createExampleBALFile(filename, P, pose1, pose2);
}

/* ************************************************************************* */

void create5PointExample2() {

  // Create two cameras poses
  Rot3 aRb = Rot3::Yaw(M_PI_2);
  Point3 aTb(10, 0, 0);
  Pose3 pose1, pose2(aRb, aTb);

  // Create test data, we need at least 5 points
  vector<Point3> P;
  P += Point3(0, 0, 100), Point3(-10, 0, 100), Point3(10, 0, 100), //
  Point3(0, 50, 50), Point3(0, -50, 50), Point3(-20, 0, 80), Point3(20, -50, 80);

  // Assumes example is run in ${GTSAM_TOP}/build/examples
  const string filename = "../../examples/data/5pointExample2.txt";
  Cal3Bundler K(500, 0, 0);
  createExampleBALFile(filename, P, pose1, pose2,K);
}

/* ************************************************************************* */

int main(int argc, char* argv[]) {
  create5PointExample1();
  create5PointExample2();
  return 0;
}

/* ************************************************************************* */

