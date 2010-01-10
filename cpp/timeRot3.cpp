/**
 * @file    timeRot3.cpp
 * @brief   time Rot3 functions
 * @author  Frank Dellaert
 */

#include <time.h>
#include <iostream>

#include "Rot3.h"

using namespace std;
using namespace gtsam;

int main()
{
  int n = 300000;
  Vector v = Vector_(3,1.,0.,0.);

  // Rodriguez formula given axis angle
  long timeLog = clock();
  for(int i = 0; i < n; i++)
  	rodriguez(v,0.001);
  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  // Rodriguez formula given canonical coordinates
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	rodriguez(v);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  // Slow rotation matrix
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	Rot3::Rz(0.3)*Rot3::Ry(0.2)*Rot3::Rx(0.1);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  // Fast Rotation matrix
  timeLog = clock();
  for(int i = 0; i < n; i++)
  	Rot3::RzRyRx(0.1,0.2,0.3);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  return 0;
}
