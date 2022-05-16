/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file  testInvDepthFactor3.cpp
 *  @brief Unit tests inverse depth parametrization
 *
 *  @author cbeall3
 *  @author Dominik Van Opdenbosch
 *  @date   Apr 13, 2012
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam_unstable/slam/InvDepthFactor3.h>

using namespace std;
using namespace gtsam;

static Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
static SharedNoiseModel sigma(noiseModel::Unit::Create(2));

// camera pose at (0,0,1) looking straight along the x-axis.
Pose3 level_pose = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
PinholeCamera<Cal3_S2> level_camera(level_pose, *K);

typedef InvDepthFactor3<Pose3, Vector5, double> InverseDepthFactor;
typedef NonlinearEquality<Pose3> PoseConstraint;

Matrix factorError(const Pose3& pose, const Vector5& point, double invDepth,
                     const InverseDepthFactor& factor) {
  return factor.evaluateError(pose, point, invDepth);
}

/* ************************************************************************* */
TEST( InvDepthFactor, optimize) {

  // landmark 5 meters infront of camera (camera center at (0,0,1))
  Point3 landmark(5, 0, 1);

  // get expected projection using pinhole camera
  Point2 expected_uv = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  Vector5 inv_landmark((Vector(5) << 0., 0., 1., 0., 0.).finished());
  // initialize inverse depth with "incorrect" depth of 1/4
  // in reality this is 1/5, but initial depth is guessed
  double inv_depth(1./4);

  gtsam::NonlinearFactorGraph graph;
  Values initial;

  InverseDepthFactor::shared_ptr factor(new InverseDepthFactor(expected_uv, sigma,
      Symbol('x',1), Symbol('l',1), Symbol('d',1), K));
  graph.push_back(factor);
  graph += PoseConstraint(Symbol('x',1),level_pose);
  initial.insert(Symbol('x',1), level_pose);
  initial.insert(Symbol('l',1), inv_landmark);
  initial.insert(Symbol('d',1), inv_depth);

  LevenbergMarquardtParams lmParams;
  Values result = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();

  // with a single factor the incorrect initialization of 1/4 should not move!
  EXPECT(assert_equal(initial, result, 1e-9));

  /// Add a second camera

  // add a camera 2 meters to the right
  Pose3 right_pose = level_pose * Pose3(Rot3(), Point3(2,0,0));
  PinholeCamera<Cal3_S2> right_camera(right_pose, *K);

  // projection measurement of landmark into right camera
  // this measurement disagrees with the depth initialization
  // and will push it to 1/5
  Point2 right_uv = right_camera.project(landmark);

  InverseDepthFactor::shared_ptr factor1(new InverseDepthFactor(right_uv, sigma,
      Symbol('x',2), Symbol('l',1),Symbol('d',1),K));
  graph.push_back(factor1);

  graph += PoseConstraint(Symbol('x',2),right_pose);

  initial.insert(Symbol('x',2), right_pose);

  Values result2 = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();

  Point3 result2_lmk = InvDepthCamera3<Cal3_S2>::invDepthTo3D(
      result2.at<Vector5>(Symbol('l',1)),
      result2.at<double>(Symbol('d',1)));
  EXPECT(assert_equal(landmark, result2_lmk, 1e-9));

  // TODO: need to add priors to make this work with
  //    Values result2 = optimize<NonlinearFactorGraph>(graph, initial,
  //      NonlinearOptimizationParameters(),MULTIFRONTAL, GN);
}


/* ************************************************************************* */
TEST( InvDepthFactor, Jacobian3D ) {

  // landmark 5 meters infront of camera (camera center at (0,0,1))
  Point3 landmark(5, 0, 1);

  // get expected projection using pinhole camera
  Point2 expected_uv = level_camera.project(landmark);

  // get expected landmark representation using backprojection
  double inv_depth;
  Vector5 inv_landmark;
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  std::tie(inv_landmark, inv_depth) = inv_camera.backproject(expected_uv, 5);
  Vector5 expected_inv_landmark((Vector(5) << 0., 0., 1., 0., 0.).finished());

  CHECK(assert_equal(expected_inv_landmark, inv_landmark, 1e-6));
  CHECK(assert_equal(inv_depth, 1./5, 1e-6));

  Symbol poseKey('x',1);
  Symbol pointKey('l',1);
  Symbol invDepthKey('d',1);
  InverseDepthFactor factor(expected_uv, sigma, poseKey, pointKey, invDepthKey, K);

  std::vector<Matrix> actualHs(3);
  factor.unwhitenedError({{poseKey, genericValue(level_pose)},
                          {pointKey, genericValue(inv_landmark)},
                          {invDepthKey,genericValue(inv_depth)}},
                         actualHs);

  const Matrix& H1Actual = actualHs.at(0);
  const Matrix& H2Actual = actualHs.at(1);
  const Matrix& H3Actual = actualHs.at(2);

  // Use numerical derivatives to verify the Jacobians
  Matrix H1Expected, H2Expected, H3Expected;

  std::function<Matrix(const Pose3 &, const Vector5 &, const double &)>
      func = std::bind(&factorError, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, factor);
  H1Expected = numericalDerivative31(func, level_pose, inv_landmark, inv_depth);
  H2Expected = numericalDerivative32(func, level_pose, inv_landmark, inv_depth);
  H3Expected = numericalDerivative33(func, level_pose, inv_landmark, inv_depth);

  // Verify the Jacobians
  CHECK(assert_equal(H1Expected, H1Actual, 1e-6))
  CHECK(assert_equal(H2Expected, H2Actual, 1e-6))
  CHECK(assert_equal(H3Expected, H3Actual, 1e-6))
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
