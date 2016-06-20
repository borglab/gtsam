/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeStereoCamera.cpp
 * @brief   time StereoCamera derivatives
 * @author  Frank Dellaert
 */

#include <time.h>
#include <iostream>

#include <gtsam/geometry/StereoCamera.h>

using namespace std;
using namespace gtsam;

int main()
{
  int n = 100000;

  const Pose3 pose1(Rot3(Vector3(1, -1, -1).asDiagonal()), Point3(0, 0, 0.5));

  const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1500, 1500, 0, 320, 240, 0.5));
  const StereoCamera camera(pose1, K);
  const Point3 point1(-0.08,-0.08, 0.0);

  Matrix computed1, computed2;
  long timeLog = clock();
  for(int i = 0; i < n; i++)
    camera.project(point1, computed1, computed2);
  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << ((double)n/seconds) << " calls/second" << endl;
  cout << ((double)seconds*1000000/n) << " musecs/call" << endl;

  return 0;
}
