/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeRot3.cpp
 * @brief   time Rot3 functions
 * @author  Frank Dellaert
 */

#include <time.h>
#include <iostream>

#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

#define TEST(TITLE,STATEMENT) \
	cout << TITLE << endl;\
	timeLog = clock();\
	for(int i = 0; i < n; i++)\
	STATEMENT;\
	timeLog2 = clock();\
	seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;\
	cout << seconds << " seconds" << endl;\
	cout << ((double)n/seconds) << " calls/second" << endl;

int main()
{
  int n = 300000; long timeLog, timeLog2; double seconds;
  Vector v = Vector_(3,1.0,0.0,0.0);//0.1,0.2,-0.1);
  Rot3 R = Rot3::rodriguez(0.1, 0.4, 0.2), R2 = R.retract(v);

  TEST("Rodriguez formula given axis angle", Rot3::rodriguez(v,0.001))
  TEST("Rodriguez formula given canonical coordinates", Rot3::rodriguez(v))
  TEST("Expmap", R*Rot3::Expmap(v))
  TEST("Logmap", Rot3::Logmap(R))
  TEST("Retract", R.retract(v))
  TEST("localCoordinates", R.localCoordinates(R2))
  TEST("Slow rotation matrix",Rot3::Rz(0.3)*Rot3::Ry(0.2)*Rot3::Rx(0.1))
  TEST("Fast Rotation matrix", Rot3::RzRyRx(0.1,0.2,0.3))

  return 0;
}
