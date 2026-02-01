/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timePinholeCamera.cpp
 * @brief   time PinholeCamera derivatives
 * @author  Frank Dellaert
 */

#include <time.h>
#include <iostream>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>

using namespace std;
using namespace gtsam;

int main()
{
  int n = 1e6;

  const Pose3 pose1(Rot3(Vector3(1, -1, -1).asDiagonal()), Point3(0, 0, 0.5));

  static Cal3Bundler K(500, 1e-3, 2.0*1e-3);
  const PinholeCamera<Cal3Bundler> camera(pose1,K);
  const Point3 point1(-0.08,-0.08, 0.0);

  /**
   * NOTE: because we only have combined derivative functions now,
   * parts of this test are no longer useful.
   */

  // Oct 12 2013, iMac 3.06GHz Core i3
  // Original:          0.14737 musecs/call
  // After collapse:    0.11469 musecs/call
  // Cal3DS2:           0.14201 musecs/call
  // After Cal3DS2 fix: 0.12231 musecs/call
  // Cal3Bundler:       0.12000 musecs/call
  // Cal3Bundler fix:   0.14637 musecs/call
  // June 24 2014, Macbook Pro 2.3GHz Core i7
  // GTSAM 3.1:         0.04295 musecs/call
  // After project fix: 0.04193 musecs/call

  {
    long timeLog = clock();
    for(int i = 0; i < n; i++)
      camera.project(point1);
    long timeLog2 = clock();
    double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
    cout << ((double)seconds*1e9/n) << " nanosecs/call" << endl;
  }

  // Oct 12 2014, Macbook Air
  {
    long timeLog = clock();
    Point2 measurement(0,0);
    for(int i = 0; i < n; i++)
      camera.project(point1)-measurement;
    long timeLog2 = clock();
    double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
    cout << ((double)seconds*1e9/n) << " nanosecs/call" << endl;
  }

  // Oct 12 2013, iMac 3.06GHz Core i3
  // Original:          3.8720 musecs/call
  // After collapse:    2.6269 musecs/call
  // Cal3DS2:           4.3330 musecs/call
  // After Cal3DS2 fix: 3.2857 musecs/call
  // Cal3Bundler:       2.6556 musecs/call
  // Cal3Bundler fix:   2.1613 musecs/call
  // June 24 2014, Macbook Pro 2.3GHz Core i7
  // GTSAM 3.1:         0.2322 musecs/call
  // After project fix: 0.2094 musecs/call
  {
    Matrix Dpose, Dpoint;
    long timeLog = clock();
    for(int i = 0; i < n; i++)
      camera.project(point1, Dpose, Dpoint, {});
    long timeLog2 = clock();
    double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
    cout << ((double)seconds*1e9/n) << " nanosecs/call" << endl;
  }

  // Oct 12 2013, iMac 3.06GHz Core i3
  // Original:          4.0119 musecs/call
  // After collapse:    2.5698 musecs/call
  // Cal3DS2:           4.8325 musecs/call
  // After Cal3DS2 fix: 3.4483 musecs/call
  // Cal3Bundler:       2.5112 musecs/call
  // Cal3Bundler fix:   2.0946 musecs/call
  // June 24 2014, Macbook Pro 2.3GHz Core i7
  // GTSAM 3.1:         0.2294 musecs/call
  // After project fix: 0.2093 nanosecs/call
  {
    Matrix Dpose, Dpoint, Dcal;
    long timeLog = clock();
    for(int i = 0; i < n; i++)
      camera.project(point1, Dpose, Dpoint, Dcal);
    long timeLog2 = clock();
    double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
    cout << ((double)seconds*1e9/n) << " nanosecs/call" << endl;
  }

  return 0;
}
