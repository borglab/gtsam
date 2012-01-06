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

int main()
{
  int n = 300000;
  Vector v = Vector_(3,1.,0.,0.);
  Rot3 R = Rot3::rodriguez(0.1, 0.4, 0.2);

  cout << "Rodriguez formula given axis angle" << endl;
  long timeLog = clock();
  for(int i = 0; i < n; i++)
  	Rot3::rodriguez(v,0.001);
  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  cout << "Rodriguez formula given canonical coordinates" << endl;
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	Rot3::rodriguez(v);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  cout << "Exmpap" << endl;
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	R*Rot3::Expmap(v);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  cout << "Retract" << endl;
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	R.retract(v);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  cout << "Slow rotation matrix" << endl;
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	Rot3::Rz(0.3)*Rot3::Ry(0.2)*Rot3::Rx(0.1);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  cout << "Fast Rotation matrix" << endl;
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	Rot3::RzRyRx(0.1,0.2,0.3);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  return 0;
}
