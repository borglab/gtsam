/**
 * @file    timeCalibratedCamera.cpp
 * @brief   time CalibratedCamera derivatives
 * @author  Frank Dellaert
 */

#include <time.h>
#include <iostream>

#include "CalibratedCamera.h"

using namespace std;
using namespace gtsam;

int main()
{
  int n = 100000;
  Matrix computed;

  const Pose3 pose1(Matrix_(3,3,
  				      1., 0., 0.,
  				      0.,-1., 0.,
  				      0., 0.,-1.
  				      ),
  			      Point3(0,0,0.5));

  const CalibratedCamera camera(pose1);
  const Point3 point1(-0.08,-0.08, 0.0);

  // Aug 8, iMac 3.06GHz Core i3
  // 0.263943 seconds
  // 378870 calls/second
  // 2.63943 musecs/call

  long timeLog = clock();
  for(int i = 0; i < n; i++)
  	computed = Dproject_pose(camera, point1);
  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << ((double)n/seconds) << " calls/second" << endl;
  cout << ((double)seconds*1000000/n) << " musecs/call" << endl;

  return 0;
}
