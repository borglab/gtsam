/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeCameraExpression.cpp
 * @brief   time CalibratedCamera derivatives
 * @author  Frank Dellaert
 * @date    October 3, 2014
 */

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include "timeLinearize.h"

using namespace std;
using namespace gtsam;

#define time timeSingleThreaded

boost::shared_ptr<Cal3_S2> fixedK(new Cal3_S2());

Point2 myProject(const Pose3& pose, const Point3& point,
    OptionalJacobian<2,6> H1, OptionalJacobian<2,3> H2) {
  PinholeCamera<Cal3_S2> camera(pose, *fixedK);
  return camera.project(point, H1, H2, {});
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

  // UNCALIBRATED

  // Dedicated factor
  // Oct 3, 2014, Macbook Air
  // 4.2 musecs/call
  NonlinearFactor::shared_ptr f1 =
      boost::make_shared<GeneralSFMFactor2<Cal3_S2> >(z, model, 1, 2, 3);
  time("GeneralSFMFactor2<Cal3_S2>  : ", f1, values);

  // ExpressionFactor
  // Oct 3, 2014, Macbook Air
  // 20.3 musecs/call
  NonlinearFactor::shared_ptr f2 =
      boost::make_shared<ExpressionFactor<Point2> >(model, z,
          uncalibrate(K, project(transformTo(x, p))));
  time("Bin(Leaf,Un(Bin(Leaf,Leaf))): ", f2, values);

  // ExpressionFactor ternary
  // Oct 3, 2014, Macbook Air
  // 20.3 musecs/call
  NonlinearFactor::shared_ptr f3 =
      boost::make_shared<ExpressionFactor<Point2> >(model, z,
          project3(x, p, K));
  time("Ternary(Leaf,Leaf,Leaf)     : ", f3, values);

  // CALIBRATED

  // Dedicated factor
  // Oct 3, 2014, Macbook Air
  // 3.4 musecs/call
  NonlinearFactor::shared_ptr g1 = boost::make_shared<
      GenericProjectionFactor<Pose3, Point3> >(z, model, 1, 2, fixedK);
  time("GenericProjectionFactor<P,P>: ", g1, values);

  // ExpressionFactor
  // Oct 3, 2014, Macbook Air
  // 16.0 musecs/call
  NonlinearFactor::shared_ptr g2 =
      boost::make_shared<ExpressionFactor<Point2> >(model, z,
          uncalibrate(Cal3_S2_(*fixedK), project(transformTo(x, p))));
  time("Bin(Cnst,Un(Bin(Leaf,Leaf))): ", g2, values);

  // ExpressionFactor, optimized
  // Oct 3, 2014, Macbook Air
  // 9.0 musecs/call
  NonlinearFactor::shared_ptr g3 =
      boost::make_shared<ExpressionFactor<Point2> >(model, z,
          Point2_(myProject, x, p));
  time("Binary(Leaf,Leaf)           : ", g3, values);
  return 0;
}
