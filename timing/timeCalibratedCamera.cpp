/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeCalibratedCamera.cpp
 * @brief   time CalibratedCamera derivatives
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <ctime>
#include <iostream>

using namespace std;
using namespace gtsam;

int main()
{
  int n = 100000;

  const Pose3 pose1(Rot3(Vector3(1, -1, -1).asDiagonal()), Point3(0, 0, 0.5));

  const CalibratedCamera camera(pose1);
  const Point3 point1(-0.08,-0.08, 0.0);

  // Aug 8, iMac 3.06GHz Core i3
  //  371153 calls/second
  //  2.69431 musecs/call
  // AFTER collapse:
  //  1.10733e+06 calls/second
  //  0.90307 musecs/call
  {
    Matrix computed1, computed2;
    long timeLog = clock();
    for(int i = 0; i < n; i++)
      camera.project(point1, computed1, computed2);
    long timeLog2 = clock();
    double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
    cout << ((double)n/seconds) << " calls/second" << endl;
    cout << ((double)seconds*1000000/n) << " musecs/call" << endl;
  }

  return 0;
}
