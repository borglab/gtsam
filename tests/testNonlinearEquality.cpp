/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testNonlinearEquality.cpp
 * @author Alex Cunningham
 */

#include <tests/simulated2DConstraints.h>

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

namespace eq2D = simulated2D::equality_constraints;

static const double tol = 1e-5;

typedef PriorFactor<Pose2> PosePrior;
typedef NonlinearEquality<Pose2> PoseNLE;
typedef std::shared_ptr<PoseNLE> shared_poseNLE;

static Symbol key('x', 1);

//******************************************************************************
TEST ( NonlinearEquality, linearization ) {
  Pose2 value = Pose2(2.1, 1.0, 2.0);
  Values linearize;
  linearize.insert(key, value);

  // create a nonlinear equality constraint
  shared_poseNLE nle(new PoseNLE(key, value));

  // check linearize
  SharedDiagonal constraintModel = noiseModel::Constrained::All(3);
  JacobianFactor expLF(key, I_3x3, Z_3x1, constraintModel);
  GaussianFactor::shared_ptr actualLF = nle->linearize(linearize);
  EXPECT(assert_equal((const GaussianFactor&)expLF, *actualLF));
}

//******************************************************************************
TEST ( NonlinearEquality, linearization_pose ) {

  Symbol key('x', 1);
  Pose2 value;
  Values config;
  config.insert(key, value);

  // create a nonlinear equality constraint
  shared_poseNLE nle(new PoseNLE(key, value));

  GaussianFactor::shared_ptr actualLF = nle->linearize(config);
  EXPECT(true);
}

//******************************************************************************
TEST ( NonlinearEquality, linearization_fail ) {
  Pose2 value = Pose2(2.1, 1.0, 2.0);
  Pose2 wrong = Pose2(2.1, 3.0, 4.0);
  Values bad_linearize;
  bad_linearize.insert(key, wrong);

  // create a nonlinear equality constraint
  shared_poseNLE nle(new PoseNLE(key, value));

  // check linearize to ensure that it fails for bad linearization points
  CHECK_EXCEPTION(nle->linearize(bad_linearize), std::invalid_argument);
}

//******************************************************************************
TEST ( NonlinearEquality, linearization_fail_pose ) {

  Symbol key('x', 1);
  Pose2 value(2.0, 1.0, 2.0), wrong(2.0, 3.0, 4.0);
  Values bad_linearize;
  bad_linearize.insert(key, wrong);

  // create a nonlinear equality constraint
  shared_poseNLE nle(new PoseNLE(key, value));

  // check linearize to ensure that it fails for bad linearization points
  CHECK_EXCEPTION(nle->linearize(bad_linearize), std::invalid_argument);
}

//******************************************************************************
TEST ( NonlinearEquality, linearization_fail_pose_origin ) {

  Symbol key('x', 1);
  Pose2 value, wrong(2.0, 3.0, 4.0);
  Values bad_linearize;
  bad_linearize.insert(key, wrong);

  // create a nonlinear equality constraint
  shared_poseNLE nle(new PoseNLE(key, value));

  // check linearize to ensure that it fails for bad linearization points
  CHECK_EXCEPTION(nle->linearize(bad_linearize), std::invalid_argument);
}

//******************************************************************************
TEST ( NonlinearEquality, error ) {
  Pose2 value = Pose2(2.1, 1.0, 2.0);
  Pose2 wrong = Pose2(2.1, 3.0, 4.0);
  Values feasible, bad_linearize;
  feasible.insert(key, value);
  bad_linearize.insert(key, wrong);

  // create a nonlinear equality constraint
  shared_poseNLE nle(new PoseNLE(key, value));

  // check error function outputs
  Vector actual = nle->unwhitenedError(feasible);
  EXPECT(assert_equal(actual, Z_3x1));

  actual = nle->unwhitenedError(bad_linearize);
  EXPECT(
      assert_equal(actual, Vector::Constant(3, std::numeric_limits<double>::infinity())));
}

//******************************************************************************
TEST ( NonlinearEquality, equals ) {
  Pose2 value1 = Pose2(2.1, 1.0, 2.0);
  Pose2 value2 = Pose2(2.1, 3.0, 4.0);

  // create some constraints to compare
  shared_poseNLE nle1(new PoseNLE(key, value1));
  shared_poseNLE nle2(new PoseNLE(key, value1));
  shared_poseNLE nle3(new PoseNLE(key, value2));

  // verify
  EXPECT(nle1->equals(*nle2));
  // basic equality = true
  EXPECT(nle2->equals(*nle1));
  // test symmetry of equals()
  EXPECT(!nle1->equals(*nle3));
  // test config
}

//******************************************************************************
TEST ( NonlinearEquality, allow_error_pose ) {
  Symbol key1('x', 1);
  Pose2 feasible1(1.0, 2.0, 3.0);
  double error_gain = 500.0;
  PoseNLE nle(key1, feasible1, error_gain);

  // the unwhitened error should provide logmap to the feasible state
  Pose2 badPoint1(0.0, 2.0, 3.0);
  Vector actVec = nle.evaluateError(badPoint1);
  Vector expVec = Vector3(-0.989992, -0.14112, 0.0);
  EXPECT(assert_equal(expVec, actVec, 1e-5));

  // the actual error should have a gain on it
  Values config;
  config.insert(key1, badPoint1);
  double actError = nle.error(config);
  DOUBLES_EQUAL(500.0, actError, 1e-9);

  // check linearization
  GaussianFactor::shared_ptr actLinFactor = nle.linearize(config);
  Matrix A1 = I_3x3;
  Vector b = expVec;
  SharedDiagonal model = noiseModel::Constrained::All(3);
  GaussianFactor::shared_ptr expLinFactor(
      new JacobianFactor(key1, A1, b, model));
  EXPECT(assert_equal(*expLinFactor, *actLinFactor, 1e-5));
}

//******************************************************************************
TEST ( NonlinearEquality, allow_error_optimize ) {
  Symbol key1('x', 1);
  Pose2 feasible1(1.0, 2.0, 3.0);
  double error_gain = 500.0;
  PoseNLE nle(key1, feasible1, error_gain);

  // add to a graph
  NonlinearFactorGraph graph;
  graph += nle;

  // initialize away from the ideal
  Pose2 initPose(0.0, 2.0, 3.0);
  Values init;
  init.insert(key1, initPose);

  // optimize
  Ordering ordering;
  ordering.push_back(key1);
  Values result = LevenbergMarquardtOptimizer(graph, init, ordering).optimize();

  // verify
  Values expected;
  expected.insert(key1, feasible1);
  EXPECT(assert_equal(expected, result));
}

//******************************************************************************
TEST ( NonlinearEquality, allow_error_optimize_with_factors ) {

  // create a hard constraint
  Symbol key1('x', 1);
  Pose2 feasible1(1.0, 2.0, 3.0);

  // initialize away from the ideal
  Values init;
  Pose2 initPose(0.0, 2.0, 3.0);
  init.insert(key1, initPose);

  double error_gain = 500.0;
  PoseNLE nle(key1, feasible1, error_gain);

  // create a soft prior that conflicts
  PosePrior prior(key1, initPose, noiseModel::Isotropic::Sigma(3, 0.1));

  // add to a graph
  NonlinearFactorGraph graph;
  graph += nle;
  graph += prior;

  // optimize
  Ordering ordering;
  ordering.push_back(key1);
  Values actual = LevenbergMarquardtOptimizer(graph, init, ordering).optimize();

  // verify
  Values expected;
  expected.insert(key1, feasible1);
  EXPECT(assert_equal(expected, actual));
}

//******************************************************************************
static SharedDiagonal hard_model = noiseModel::Constrained::All(2);
static SharedDiagonal soft_model = noiseModel::Isotropic::Sigma(2, 1.0);

//******************************************************************************
TEST( testNonlinearEqualityConstraint, unary_basics ) {
  Point2 pt(1.0, 2.0);
  Symbol key1('x', 1);
  double mu = 1000.0;
  eq2D::UnaryEqualityConstraint constraint(pt, key, mu);

  Values config1;
  config1.insert(key, pt);
  EXPECT(constraint.active(config1));
  EXPECT(assert_equal(Z_2x1, constraint.evaluateError(pt), tol));
  EXPECT(assert_equal(Z_2x1, constraint.unwhitenedError(config1), tol));
  EXPECT_DOUBLES_EQUAL(0.0, constraint.error(config1), tol);

  Values config2;
  Point2 ptBad1(2.0, 2.0);
  config2.insert(key, ptBad1);
  EXPECT(constraint.active(config2));
  EXPECT(
      assert_equal(Vector2(1.0, 0.0), constraint.evaluateError(ptBad1), tol));
  EXPECT(
      assert_equal(Vector2(1.0, 0.0), constraint.unwhitenedError(config2), tol));
  EXPECT_DOUBLES_EQUAL(500.0, constraint.error(config2), tol);
}

//******************************************************************************
TEST( testNonlinearEqualityConstraint, unary_linearization ) {
  Point2 pt(1.0, 2.0);
  Symbol key1('x', 1);
  double mu = 1000.0;
  eq2D::UnaryEqualityConstraint constraint(pt, key, mu);

  Values config1;
  config1.insert(key, pt);
  GaussianFactor::shared_ptr actual1 = constraint.linearize(config1);
  GaussianFactor::shared_ptr expected1(
      new JacobianFactor(key, I_2x2, Z_2x1, hard_model));
  EXPECT(assert_equal(*expected1, *actual1, tol));

  Values config2;
  Point2 ptBad(2.0, 2.0);
  config2.insert(key, ptBad);
  GaussianFactor::shared_ptr actual2 = constraint.linearize(config2);
  GaussianFactor::shared_ptr expected2(
      new JacobianFactor(key, I_2x2, Vector2(-1.0, 0.0), hard_model));
  EXPECT(assert_equal(*expected2, *actual2, tol));
}

//******************************************************************************
TEST( testNonlinearEqualityConstraint, unary_simple_optimization ) {
  // create a single-node graph with a soft and hard constraint to
  // ensure that the hard constraint overrides the soft constraint
  Point2 truth_pt(1.0, 2.0);
  Symbol key('x', 1);
  double mu = 10.0;
  eq2D::UnaryEqualityConstraint::shared_ptr constraint(
      new eq2D::UnaryEqualityConstraint(truth_pt, key, mu));

  Point2 badPt(100.0, -200.0);
  simulated2D::Prior::shared_ptr factor(
      new simulated2D::Prior(badPt, soft_model, key));

  NonlinearFactorGraph graph;
  graph.push_back(constraint);
  graph.push_back(factor);

  Values initValues;
  initValues.insert(key, badPt);

  // verify error values
  EXPECT(constraint->active(initValues));

  Values expected;
  expected.insert(key, truth_pt);
  EXPECT(constraint->active(expected));
  EXPECT_DOUBLES_EQUAL(0.0, constraint->error(expected), tol);

  Values actual = LevenbergMarquardtOptimizer(graph, initValues).optimize();
  EXPECT(assert_equal(expected, actual, tol));
}

//******************************************************************************
TEST( testNonlinearEqualityConstraint, odo_basics ) {
  Point2 x1(1.0, 2.0), x2(2.0, 3.0), odom(1.0, 1.0);
  Symbol key1('x', 1), key2('x', 2);
  double mu = 1000.0;
  eq2D::OdoEqualityConstraint constraint(odom, key1, key2, mu);

  Values config1;
  config1.insert(key1, x1);
  config1.insert(key2, x2);
  EXPECT(constraint.active(config1));
  EXPECT(assert_equal(Z_2x1, constraint.evaluateError(x1, x2), tol));
  EXPECT(assert_equal(Z_2x1, constraint.unwhitenedError(config1), tol));
  EXPECT_DOUBLES_EQUAL(0.0, constraint.error(config1), tol);

  Values config2;
  Point2 x1bad(2.0, 2.0);
  Point2 x2bad(2.0, 2.0);
  config2.insert(key1, x1bad);
  config2.insert(key2, x2bad);
  EXPECT(constraint.active(config2));
  EXPECT(
      assert_equal(Vector2(-1.0, -1.0), constraint.evaluateError(x1bad, x2bad), tol));
  EXPECT(
      assert_equal(Vector2(-1.0, -1.0), constraint.unwhitenedError(config2), tol));
  EXPECT_DOUBLES_EQUAL(1000.0, constraint.error(config2), tol);
}

//******************************************************************************
TEST( testNonlinearEqualityConstraint, odo_linearization ) {
  Point2 x1(1.0, 2.0), x2(2.0, 3.0), odom(1.0, 1.0);
  Symbol key1('x', 1), key2('x', 2);
  double mu = 1000.0;
  eq2D::OdoEqualityConstraint constraint(odom, key1, key2, mu);

  Values config1;
  config1.insert(key1, x1);
  config1.insert(key2, x2);
  GaussianFactor::shared_ptr actual1 = constraint.linearize(config1);
  GaussianFactor::shared_ptr expected1(
      new JacobianFactor(key1, -I_2x2, key2, I_2x2, Z_2x1,
          hard_model));
  EXPECT(assert_equal(*expected1, *actual1, tol));

  Values config2;
  Point2 x1bad(2.0, 2.0);
  Point2 x2bad(2.0, 2.0);
  config2.insert(key1, x1bad);
  config2.insert(key2, x2bad);
  GaussianFactor::shared_ptr actual2 = constraint.linearize(config2);
  GaussianFactor::shared_ptr expected2(
      new JacobianFactor(key1, -I_2x2, key2, I_2x2, Vector2(1.0, 1.0),
          hard_model));
  EXPECT(assert_equal(*expected2, *actual2, tol));
}

//******************************************************************************
TEST( testNonlinearEqualityConstraint, odo_simple_optimize ) {
  // create a two-node graph, connected by an odometry constraint, with
  // a hard prior on one variable, and a conflicting soft prior
  // on the other variable - the constraints should override the soft constraint
  Point2 truth_pt1(1.0, 2.0), truth_pt2(3.0, 2.0);
  Symbol key1('x', 1), key2('x', 2);

  // hard prior on x1
  eq2D::UnaryEqualityConstraint::shared_ptr constraint1(
      new eq2D::UnaryEqualityConstraint(truth_pt1, key1));

  // soft prior on x2
  Point2 badPt(100.0, -200.0);
  simulated2D::Prior::shared_ptr factor(
      new simulated2D::Prior(badPt, soft_model, key2));

  // odometry constraint
  eq2D::OdoEqualityConstraint::shared_ptr constraint2(
      new eq2D::OdoEqualityConstraint(truth_pt2-truth_pt1, key1, key2));

  NonlinearFactorGraph graph;
  graph.push_back(constraint1);
  graph.push_back(constraint2);
  graph.push_back(factor);

  Values initValues;
  initValues.insert(key1, Point2(0,0));
  initValues.insert(key2, badPt);

  Values actual = LevenbergMarquardtOptimizer(graph, initValues).optimize();
  Values expected;
  expected.insert(key1, truth_pt1);
  expected.insert(key2, truth_pt2);
  CHECK(assert_equal(expected, actual, tol));
}

//******************************************************************************
TEST (testNonlinearEqualityConstraint, two_pose ) {
  /*
   * Determining a ground truth linear system
   * with two poses seeing one landmark, with each pose
   * constrained to a particular value
   */

  NonlinearFactorGraph graph;

  Symbol x1('x', 1), x2('x', 2);
  Symbol l1('l', 1), l2('l', 2);
  Point2 pt_x1(1.0, 1.0), pt_x2(5.0, 6.0);
  graph += eq2D::UnaryEqualityConstraint(pt_x1, x1);
  graph += eq2D::UnaryEqualityConstraint(pt_x2, x2);

  Point2 z1(0.0, 5.0);
  SharedNoiseModel sigma(noiseModel::Isotropic::Sigma(2, 0.1));
  graph += simulated2D::Measurement(z1, sigma, x1, l1);

  Point2 z2(-4.0, 0.0);
  graph += simulated2D::Measurement(z2, sigma, x2, l2);

  graph += eq2D::PointEqualityConstraint(l1, l2);

  Values initialEstimate;
  initialEstimate.insert(x1, pt_x1);
  initialEstimate.insert(x2, Point2(0,0));
  initialEstimate.insert(l1, Point2(1.0, 6.0)); // ground truth
  initialEstimate.insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame

  Values actual =
      LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

  Values expected;
  expected.insert(x1, pt_x1);
  expected.insert(l1, Point2(1.0, 6.0));
  expected.insert(l2, Point2(1.0, 6.0));
  expected.insert(x2, Point2(5.0, 6.0));
  CHECK(assert_equal(expected, actual, 1e-5));
}

//******************************************************************************
TEST (testNonlinearEqualityConstraint, map_warp ) {
  // get a graph
  NonlinearFactorGraph graph;

  // keys
  Symbol x1('x', 1), x2('x', 2);
  Symbol l1('l', 1), l2('l', 2);

  // constant constraint on x1
  Point2 pose1(1.0, 1.0);
  graph += eq2D::UnaryEqualityConstraint(pose1, x1);

  SharedDiagonal sigma = noiseModel::Isotropic::Sigma(2, 0.1);

  // measurement from x1 to l1
  Point2 z1(0.0, 5.0);
  graph += simulated2D::Measurement(z1, sigma, x1, l1);

  // measurement from x2 to l2
  Point2 z2(-4.0, 0.0);
  graph += simulated2D::Measurement(z2, sigma, x2, l2);

  // equality constraint between l1 and l2
  graph += eq2D::PointEqualityConstraint(l1, l2);

  // create an initial estimate
  Values initialEstimate;
  initialEstimate.insert(x1, Point2(1.0, 1.0));
  initialEstimate.insert(l1, Point2(1.0, 6.0));
  initialEstimate.insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame
  initialEstimate.insert(x2, Point2(0.0, 0.0)); // other pose starts at origin

  // optimize
  Values actual =
      LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

  Values expected;
  expected.insert(x1, Point2(1.0, 1.0));
  expected.insert(l1, Point2(1.0, 6.0));
  expected.insert(l2, Point2(1.0, 6.0));
  expected.insert(x2, Point2(5.0, 6.0));
  CHECK(assert_equal(expected, actual, tol));
}

//******************************************************************************
TEST (testNonlinearEqualityConstraint, stereo_constrained ) {

  // make a realistic calibration matrix
  static double fov = 60; // degrees
  static int w = 640, h = 480;
  static Cal3_S2 K(fov, w, h);
  static std::shared_ptr<Cal3_S2> shK(new Cal3_S2(K));

  // create initial estimates
  Rot3 faceTowardsY(Point3(1, 0, 0), Point3(0, 0, -1), Point3(0, 1, 0));

  Pose3 poseLeft(faceTowardsY, Point3(0, 0, 0));  // origin, left camera
  PinholeCamera<Cal3_S2> leftCamera(poseLeft, K);

  Pose3 poseRight(faceTowardsY, Point3(2, 0, 0));  // 2 units to the right
  PinholeCamera<Cal3_S2> rightCamera(poseRight, K);

  Point3 landmark(1, 5, 0); //centered between the cameras, 5 units away

  // keys
  Symbol key_x1('x', 1), key_x2('x', 2);
  Symbol key_l1('l', 1), key_l2('l', 2);

  // create graph
  NonlinearFactorGraph graph;

  // create equality constraints for poses
  graph += NonlinearEquality<Pose3>(key_x1, leftCamera.pose());
  graph += NonlinearEquality<Pose3>(key_x2, rightCamera.pose());

  // create  factors
  SharedDiagonal vmodel = noiseModel::Unit::Create(2);
  graph += GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
      leftCamera.project(landmark), vmodel, key_x1, key_l1, shK);
  graph += GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
      rightCamera.project(landmark), vmodel, key_x2, key_l2, shK);

  // add equality constraint saying there is only one point
  graph += NonlinearEquality2<Point3>(key_l1, key_l2);

  // create initial data
  Point3 landmark1(0.5, 5, 0);
  Point3 landmark2(1.5, 5, 0);

  Values initValues;
  initValues.insert(key_x1, poseLeft);
  initValues.insert(key_x2, poseRight);
  initValues.insert(key_l1, landmark1);
  initValues.insert(key_l2, landmark2);

  // optimize
  Values actual = LevenbergMarquardtOptimizer(graph, initValues).optimize();

  // create config
  Values truthValues;
  truthValues.insert(key_x1, leftCamera.pose());
  truthValues.insert(key_x2, rightCamera.pose());
  truthValues.insert(key_l1, landmark);
  truthValues.insert(key_l2, landmark);

  // check if correct
  CHECK(assert_equal(truthValues, actual, 1e-5));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
