/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timePose3.cpp
 * @brief   time Pose3 functions
 * @author  Frank Dellaert
 */

#include <time.h>
#include <iostream>

#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

#define TEST(TITLE,STATEMENT) \
	cout << endl << TITLE << endl;\
	timeLog = clock();\
	for(int i = 0; i < n; i++)\
	STATEMENT;\
	timeLog2 = clock();\
	seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;\
	cout << seconds << " seconds" << endl;\
	cout << ((double)n/seconds) << " calls/second" << endl;

int main()
{
  int n = 100000; long timeLog, timeLog2; double seconds;
	double norm=sqrt(1.0+16.0+4.0);
	double x=1.0/norm, y=4.0/norm, z=2.0/norm;
  Vector v = Vector_(6,x,y,z,0.1,0.2,-0.1);
  Pose3 T = Pose3::Expmap(Vector_(6,0.1,0.1,0.2,0.1, 0.4, 0.2)), T2 = T.retract(v);

  TEST("retract", T.retract(v))
  TEST("Expmap", T*Pose3::Expmap(v))
  TEST("localCoordinates", T.localCoordinates(T2))
  TEST("Logmap", Pose3::Logmap(T.between(T2)))

  return 0;
}
