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
  int n = 3000000;
  Vector v = Vector_(3,1.,0.,0.);

  long timeLog = clock();
  for(int i = 0; i < n; i++)
  	rodriguez(v,0.001);
  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  timeLog = clock();
  for(int i = 0; i < n; i++)
  	rodriguez(v);
  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  return 0;
}
