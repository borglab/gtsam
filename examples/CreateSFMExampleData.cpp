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

#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/slam/dataset.h>

#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */

void createExampleBALFile(const string& filename, const vector<Point3>& points,
                          const Pose3& pose1, const Pose3& pose2,
                          const Cal3Bundler& K = Cal3Bundler()) {
  // Class that will gather all data
  SfmData data;
  // Create two cameras and add them to data
  data.cameras.push_back(SfmCamera(pose1, K));
  data.cameras.push_back(SfmCamera(pose2, K));

  for (const Point3& p : points) {
    // Create the track
    SfmTrack track;
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
  vector<Point3> points = {
      {0, 0, 1}, {-0.1, 0, 1}, {0.1, 0, 1}, {0, 0.5, 0.5}, {0, -0.5, 0.5}};

  // Assumes example is run in ${GTSAM_TOP}/build/examples
  const string filename = "../../examples/Data/5pointExample1.txt";
  createExampleBALFile(filename, points, pose1, pose2);
}

/* ************************************************************************* */

void create5PointExample2() {
  // Create two cameras poses
  Rot3 aRb = Rot3::Yaw(M_PI_2);
  Point3 aTb(10, 0, 0);
  Pose3 pose1, pose2(aRb, aTb);

  // Create test data, we need at least 5 points
  vector<Point3> points = {{0, 0, 100},  {-10, 0, 100}, {10, 0, 100},  //
                           {0, 50, 50},  {0, -50, 50},  {-20, 0, 80},  //
                           {20, -50, 80}};

  // Assumes example is run in ${GTSAM_TOP}/build/examples
  const string filename = "../../examples/Data/5pointExample2.txt";
  Cal3Bundler K(500, 0, 0);
  createExampleBALFile(filename, points, pose1, pose2, K);
}

/* ************************************************************************* */

void create18PointExample1() {
  // Create two cameras poses
  Rot3 aRb = Rot3::Yaw(M_PI_2);
  Point3 aTb(0.1, 0, 0);
  Pose3 pose1, pose2(aRb, aTb);

  // Create test data, we need 15 points
  vector<Point3> points = {
      {0, 0, 1},         {-0.1, 0, 1},      {0.1, 0, 1},       //
      {0, 0.5, 0.5},     {0, -0.5, 0.5},    {-1, -0.5, 2},     //
      {-1, 0.5, 2},      {0.25, -0.5, 1.5}, {0.25, 0.5, 1.5},  //
      {-0.1, -0.5, 0.5}, {0.1, -0.5, 1},    {0.1, 0.5, 1},     //
      {-0.1, 0, 0.5},    {-0.1, 0.5, 0.5},  {0, 0, 0.5},       //
      {0.1, -0.5, 0.5},  {0.1, 0, 0.5},     {0.1, 0.5, 0.5}};

  // Assumes example is run in ${GTSAM_TOP}/build/examples
  const string filename = "../../examples/Data/18pointExample1.txt";
  createExampleBALFile(filename, points, pose1, pose2);
}

int main(int argc, char* argv[]) {
  create5PointExample1();
  create5PointExample2();
  create18PointExample1();
  return 0;
}

/* ************************************************************************* */
