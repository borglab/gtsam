/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeOneCameraExpression.cpp
 * @brief   time CalibratedCamera derivatives
 * @author  Frank Dellaert
 * @date    October 3, 2014
 */

#include <gtsam_unstable/slam/expressions.h>
#include <gtsam_unstable/nonlinear/BADFactor.h>

#include <time.h>
#include <iostream>
#include <iomanip>      // std::setprecision
using namespace std;
using namespace gtsam;

static const int n = 500000;

void time(const NonlinearFactor& f, const Values& values) {
  long timeLog = clock();
  GaussianFactor::shared_ptr gf;
  for (int i = 0; i < n; i++)
    gf = f.linearize(values);
  long timeLog2 = clock();
  double seconds = (double) (timeLog2 - timeLog) / CLOCKS_PER_SEC;
  cout << setprecision(3);
  cout << ((double) seconds * 1000000 / n) << " musecs/call" << endl;
}

int main() {

  // Create leaves
  Pose3_ x(1);
  Point3_ p(2);
  Cal3_S2_ K(3);

  // Some parameters needed
  Point2 z(-17, 30);
  SharedNoiseModel model = noiseModel::Unit::Create(2);

  // Create values
  Values values;
  values.insert(1, Pose3());
  values.insert(2, Point3(0, 0, 1));
  values.insert(3, Cal3_S2());

  // BADFactor
  // Oct 3, 2014, Macbook Air
  // 20.3 musecs/call
#define TERNARY
#ifdef TERNARY
  BADFactor<Point2> f(model, z, project3(x, p, K));
#else
  BADFactor<Point2> f(model, z, uncalibrate(K, project(transform_to(x, p))));
#endif
  time(f, values);

  return 0;
}
