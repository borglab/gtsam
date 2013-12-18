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

void create5PointExample1() {

  // Class that will gather all data
  SfM_data data;

  // Create two cameras and corresponding essential matrix E
  Rot3 aRb = Rot3::yaw(M_PI_2);
  Point3 aTb(0.1, 0, 0);
  Pose3 identity, aPb(aRb, aTb);
  data.cameras.push_back(SfM_Camera(identity));
  data.cameras.push_back(SfM_Camera(aPb));

  // Create test data, we need at least 5 points
  vector<Point3> P;
  P += Point3(0, 0, 1), Point3(-0.1, 0, 1), Point3(0.1, 0, 1), //
  Point3(0, 0.5, 0.5), Point3(0, -0.5, 0.5);

  BOOST_FOREACH(const Point3& p, P) {

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

  // Assumes example is run in ${GTSAM_TOP}/build/examples
  const string filename = "../../examples/data/5pointExample1.txt";
  writeBAL(filename, data);
}

int main(int argc, char* argv[]) {
  create5PointExample1();
  return 0;
}

/* ************************************************************************* */

