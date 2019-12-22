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

#define TEST(TITLE, STATEMENT)                                        \
  cout << endl << TITLE << endl;                                      \
  timeLog = clock();                                                  \
  for (int i = 0; i < n; i++) STATEMENT;                              \
  timeLog2 = clock();                                                 \
  seconds = static_cast<double>(timeLog2 - timeLog) / CLOCKS_PER_SEC; \
  cout << 1000 * seconds << " milliseconds" << endl;                  \
  cout << (1e9 * seconds / static_cast<double>(n)) << " nanosecs/call" << endl;

int main() {
  int n = 100000;
  clock_t timeLog, timeLog2;
  double seconds;
  // create a random direction:
  double norm = sqrt(1.0 + 16.0 + 4.0);
  double x = 1.0 / norm, y = 4.0 / norm, z = 2.0 / norm;
  Vector v = (Vector(3) << x, y, z).finished();
  Rot3 R = Rot3::Rodrigues(0.1, 0.4, 0.2), R2 = R.retract(v);

  TEST("Rodriguez formula given axis angle", Rot3::AxisAngle(v, 0.001))
  TEST("Rodriguez formula given canonical coordinates", Rot3::Rodrigues(v))
  TEST("Expmap", R * Rot3::Expmap(v))
  TEST("Retract", R.retract(v))
  TEST("Logmap", Rot3::Logmap(R.between(R2)))
  TEST("localCoordinates", R.localCoordinates(R2))
  TEST("Slow rotation matrix", Rot3::Rz(z) * Rot3::Ry(y) * Rot3::Rx(x))
  TEST("Fast Rotation matrix", Rot3::RzRyRx(x, y, z))

  return 0;
}
