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

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include "timeLinearize.h"

using namespace std;
using namespace gtsam;

#define time timeSingleThreaded

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

  // ExpressionFactor
  // Oct 3, 2014, Macbook Air
  // 20.3 musecs/call
//#define TERNARY
  NonlinearFactor::shared_ptr f = std::make_shared<ExpressionFactor<Point2> >
#ifdef TERNARY
      (model, z, project3(x, p, K));
#else
      (model, z, uncalibrate(K, project(transformTo(x, p))));
#endif
  time("timing:", f, values);

  return 0;
}
