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

#include <iostream>

#include <gtsam/base/timing.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#define TEST(TITLE,STATEMENT) \
  gttic_(TITLE); \
  for(int i = 0; i < n; i++) \
  STATEMENT; \
  gttoc_(TITLE);

int main()
{
  int n = 5000000;
  cout << "NOTE:  Times are reported for " << n << " calls" << endl;

  double norm=sqrt(1.0+16.0+4.0);
  double x=1.0/norm, y=4.0/norm, z=2.0/norm;
  Vector v = (Vector(6) << x, y, z, 0.1, 0.2, -0.1).finished();
  Pose3 T = Pose3::Expmap((Vector(6) << 0.1, 0.1, 0.2, 0.1, 0.4, 0.2).finished()), T2 = T.retract(v);
  Matrix H1,H2;

  TEST(retract, T.retract(v))
  TEST(Expmap, T*Pose3::Expmap(v))
  TEST(localCoordinates, T.localCoordinates(T2))
  TEST(between, T.between(T2))
  TEST(between_derivatives, T.between(T2,H1,H2))
  TEST(Logmap, Pose3::Logmap(T.between(T2)))

  // Print timings
  tictoc_print_();

  return 0;
}
