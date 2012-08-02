/*
 * testInvDepthFactor.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: cbeall3
 */

#include <gtsam_unstable/slam/InvDepthFactor3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Symbol.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
static Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
Pose3 level_pose = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
SimpleCamera level_camera(level_pose, *K);

/* ************************************************************************* */
TEST( InvDepthFactor, Project1) {

  // landmark 5 meters infront of camera
  Point3 landmark(5, 0, 1);

  Point2 expected_uv = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  LieVector inv_landmark(5, 1., 0., 1., 0., 0.);
  LieScalar inv_depth(1./4);
  Point2 actual_uv = inv_camera.project(inv_landmark, inv_depth);
  CHECK(assert_equal(expected_uv, actual_uv));
  CHECK(assert_equal(Point2(640,480), actual_uv));

}

/* ************************************************************************* */
TEST( InvDepthFactor, Project2) {

  // landmark 1m to the left and 1m up from camera
  // inv landmark xyz is same as camera xyz, so depth  actually doesn't matter
  Point3 landmark(1, 1, 2);
  Point2 expected = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  LieVector diag_landmark(5, 0., 0., 1., M_PI/4., atan(1/sqrt(2)));
  LieScalar inv_depth(1/sqrt(3));
  Point2 actual = inv_camera.project(diag_landmark, inv_depth);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Project3) {

  // landmark 1m to the left and 1m up from camera
  // inv depth landmark xyz at origion
  Point3 landmark(1, 1, 2);
  Point2 expected = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  LieVector diag_landmark(5, 0., 0., 0., M_PI/4., atan(2./sqrt(2)));
  LieScalar inv_depth( 1./sqrt(1+1+4));
  Point2 actual = inv_camera.project(diag_landmark, inv_depth);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Project4) {

  // landmark 4m to the left and 1m up from camera
  // inv depth landmark xyz at origion
  Point3 landmark(1, 4, 2);
  Point2 expected = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  LieVector diag_landmark(5, 0., 0., 0., atan(4/1), atan(2./sqrt(1+16)));
  LieScalar inv_depth(1./sqrt(1+16+4));
  Point2 actual = inv_camera.project(diag_landmark, inv_depth);
  CHECK(assert_equal(expected, actual));
}


/* ************************************************************************* */
Point2 project_(const Pose3& pose, const LieVector& landmark, const LieScalar& inv_depth) {
  return InvDepthCamera3<Cal3_S2>(pose,K).project(landmark, inv_depth); }

TEST( InvDepthFactor, Dproject_pose)
{
  LieVector landmark(6,0.1,0.2,0.3, 0.1,0.2);
  LieScalar inv_depth(1./4);
  Matrix expected = numericalDerivative31<Point2,Pose3,LieVector>(project_,level_pose, landmark, inv_depth);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Matrix actual;
  Point2 uv = inv_camera.project(landmark, inv_depth, actual, boost::none, boost::none);
  CHECK(assert_equal(expected,actual,1e-6));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Dproject_landmark)
{
  LieVector landmark(5,0.1,0.2,0.3, 0.1,0.2);
  LieScalar inv_depth(1./4);
  Matrix expected = numericalDerivative32<Point2,Pose3,LieVector>(project_,level_pose, landmark, inv_depth);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Matrix actual;
  Point2 uv = inv_camera.project(landmark, inv_depth, boost::none, actual, boost::none);
  CHECK(assert_equal(expected,actual,1e-7));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Dproject_inv_depth)
{
  LieVector landmark(5,0.1,0.2,0.3, 0.1,0.2);
  LieScalar inv_depth(1./4);
  Matrix expected = numericalDerivative33<Point2,Pose3,LieVector>(project_,level_pose, landmark, inv_depth);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Matrix actual;
  Point2 uv = inv_camera.project(landmark, inv_depth, boost::none, boost::none, actual);
  CHECK(assert_equal(expected,actual,1e-7));
}

/* ************************************************************************* */
TEST(InvDepthFactor, backproject)
{
  LieVector expected(5,0.,0.,1., 0.1,0.2);
  LieScalar inv_depth(1./4);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Point2 z = inv_camera.project(expected, inv_depth);

  LieVector actual_vec;
  LieScalar actual_inv;
  boost::tie(actual_vec, actual_inv) = inv_camera.backproject(z, 4);
  CHECK(assert_equal(expected,actual_vec,1e-7));
  CHECK(assert_equal(inv_depth,actual_inv,1e-7));
}

/* ************************************************************************* */
TEST(InvDepthFactor, backproject2)
{
  // backwards facing camera
  LieVector expected(5,-5.,-5.,2., 3., -0.1);
  LieScalar inv_depth(1./10);
  InvDepthCamera3<Cal3_S2> inv_camera(Pose3(Rot3::ypr(1.5,0.1, -1.5), Point3(-5, -5, 2)),K);
  Point2 z = inv_camera.project(expected, inv_depth);

  LieVector actual_vec;
  LieScalar actual_inv;
  boost::tie(actual_vec, actual_inv) = inv_camera.backproject(z, 10);
  CHECK(assert_equal(expected,actual_vec,1e-7));
  CHECK(assert_equal(inv_depth,actual_inv,1e-7));
}

static SharedNoiseModel sigma(noiseModel::Unit::Create(2));
typedef InvDepthFactor3<Pose3, LieVector, LieScalar> InverseDepthFactor;
typedef NonlinearEquality<Pose3> PoseConstraint;

/* ************************************************************************* */
TEST( InvDepthFactor, optimize) {

  // landmark 5 meters infront of camera
  Point3 landmark(5, 0, 1);

  Point2 expected_uv = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  LieVector inv_landmark(5, 1., 0., 1., 0., 0.);
  LieScalar inv_depth(1./4);

  gtsam::NonlinearFactorGraph graph;
  Values initial;

  InverseDepthFactor::shared_ptr factor(new InverseDepthFactor(expected_uv, sigma,
      Symbol('x',1), Symbol('l',1), Symbol('d',1), K));
  graph.push_back(factor);
  graph.add(PoseConstraint(Symbol('x',1),level_pose));
  initial.insert(Symbol('x',1), level_pose);
  initial.insert(Symbol('l',1), inv_landmark);
  initial.insert(Symbol('d',1), inv_depth);

  LevenbergMarquardtParams lmParams;
  Values result = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();
  CHECK(assert_equal(initial, result, 1e-9));

  /// Add a second camera

  // add a camera 1 meter to the right
  Pose3 right_pose = level_pose * Pose3(Rot3(), Point3(2,0,0));
  SimpleCamera right_camera(right_pose, *K);

  InvDepthCamera3<Cal3_S2> right_inv_camera(right_pose, K);

  Point3 landmark1(6,0,1);
  Point2 right_uv = right_camera.project(landmark1);


  InverseDepthFactor::shared_ptr factor1(new InverseDepthFactor(right_uv, sigma,
      Symbol('x',2), Symbol('l',1),Symbol('d',1),K));
  graph.push_back(factor1);

  graph.add(PoseConstraint(Symbol('x',2),right_pose));

  initial.insert(Symbol('x',2), right_pose);

  // TODO: need to add priors to make this work with
//    Values result2 = optimize<NonlinearFactorGraph>(graph, initial,
//      NonlinearOptimizationParameters(),MULTIFRONTAL, GN);

  Values result2 = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();
  Point3 l1_result2 = InvDepthCamera3<Cal3_S2>::invDepthTo3D(
      result2.at<LieVector>(Symbol('l',1)),
      result2.at<LieScalar>(Symbol('d',1)));

  CHECK(assert_equal(landmark1, l1_result2, 1e-9));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
