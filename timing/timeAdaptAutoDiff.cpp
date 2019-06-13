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

#include "timeLinearize.h"
#include <gtsam/3rdparty/ceres/example.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Point3.h>

using namespace std;
using namespace gtsam;

#define time timeMultiThreaded

int main() {

  // The DefaultChart of Camera below is laid out like Snavely's 9-dim vector
  typedef PinholeCamera<Cal3Bundler> Camera;
  typedef Expression<Point2> Point2_;
  typedef Expression<Camera> Camera_;
  typedef Expression<Point3> Point3_;

  // Create leaves
  Camera_ camera(1);
  Point3_ point(2);

  // Some parameters needed
  Point2 z(-17, 30);
  SharedNoiseModel model = noiseModel::Unit::Create(2);

  // Create values
  Values values;
  values.insert(1, Camera());
  values.insert(2, Point3(0, 0, 1));

  NonlinearFactor::shared_ptr f1, f2, f3;

  // Dedicated factor
  f1 = boost::make_shared<GeneralSFMFactor<Camera, Point3> >(z, model, 1, 2);
  time("GeneralSFMFactor<Camera>                : ", f1, values);

  // ExpressionFactor
  Point2_ expression2(camera, &Camera::project2, point);
  f3 = boost::make_shared<ExpressionFactor<Point2> >(model, z, expression2);
  time("Point2_(camera, &Camera::project, point): ", f3, values);

  // AdaptAutoDiff
  values.clear();
  values.insert(1,Vector9(Vector9::Zero()));
  values.insert(2,Vector3(0,0,1));
  typedef AdaptAutoDiff<SnavelyProjection, 2, 9, 3> AdaptedSnavely;
  Expression<Vector2> expression(AdaptedSnavely(), Expression<Vector9>(1), Expression<Vector3>(2));
  f2 = boost::make_shared<ExpressionFactor<Vector2> >(model, z, expression);
  time("Point2_(AdaptedSnavely(), camera, point): ", f2, values);

  return 0;
}
