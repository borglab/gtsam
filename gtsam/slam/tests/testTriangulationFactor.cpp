/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * testTriangulationFactor.cpp
 *
 *  Created on: July 30th, 2013
 *      Author: cbeall3
 */

#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/bind/bind.hpp>

using namespace std;
using namespace gtsam;
using namespace std::placeholders;

// Some common constants
static const boost::shared_ptr<Cal3_S2> sharedCal = //
    boost::make_shared<Cal3_S2>(1500, 1200, 0, 640, 480);

// Looking along X-axis, 1 meter above ground plane (x-y)
static const Rot3 upright = Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2);
static const Pose3 pose1 = Pose3(upright, gtsam::Point3(0, 0, 1));
PinholeCamera<Cal3_S2> camera1(pose1, *sharedCal);

// landmark ~5 meters infront of camera
static const Point3 landmark(5, 0.5, 1.2);

// 1. Project two landmarks into two cameras and triangulate
Point2 z1 = camera1.project(landmark);

//******************************************************************************
TEST( triangulation, TriangulationFactor ) {

  Key pointKey(1);
  SharedNoiseModel model;
  typedef TriangulationFactor<PinholeCamera<Cal3_S2> > Factor;
  Factor factor(camera1, z1, model, pointKey);

  // Use the factor to calculate the Jacobians
  Matrix HActual;
  factor.evaluateError(landmark, HActual);

  Matrix HExpected = numericalDerivative11<Vector,Point3>(
		  [&factor](const Point3& l) { return factor.evaluateError(l);}, landmark);

  // Verify the Jacobians are correct
  CHECK(assert_equal(HExpected, HActual, 1e-3));
}

//******************************************************************************
TEST( triangulation, TriangulationFactorStereo ) {

  Key pointKey(1);
  SharedNoiseModel model=noiseModel::Isotropic::Sigma(3,0.5);
  Cal3_S2Stereo::shared_ptr stereoCal(new Cal3_S2Stereo(1500, 1200, 0, 640, 480, 0.5));
  StereoCamera camera2(pose1, stereoCal);

  StereoPoint2 z2 = camera2.project(landmark);

  typedef TriangulationFactor<StereoCamera> Factor;
  Factor factor(camera2, z2, model, pointKey);

  // Use the factor to calculate the Jacobians
  Matrix HActual;
  factor.evaluateError(landmark, HActual);

  Matrix HExpected = numericalDerivative11<Vector, Point3>(
		  [&factor](const Point3& l) { return factor.evaluateError(l);}, landmark);

  // Verify the Jacobians are correct
  CHECK(assert_equal(HExpected, HActual, 1e-3));

  // compare same problem against expression factor
  Expression<StereoPoint2>::UnaryFunction<Point3>::type f =
      std::bind(&StereoCamera::project2, camera2, std::placeholders::_1,
                boost::none, std::placeholders::_2);
  Expression<Point3> point_(pointKey);
  Expression<StereoPoint2> project2_(f, point_);

  ExpressionFactor<StereoPoint2> eFactor(model, z2, project2_);

  Values values;
  values.insert(pointKey, landmark);

  vector<Matrix> HActual1(1), HActual2(1);
  Vector error1 = factor.unwhitenedError(values, HActual1);
  Vector error2 = eFactor.unwhitenedError(values, HActual2);
  EXPECT(assert_equal(error1, error2));
  EXPECT(assert_equal(HActual1[0], HActual2[0]));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
