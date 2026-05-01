/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testWnoaFactor.cpp
 * @brief   Unit test for WNOA Factor
 * @author  Connor Holmes
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/WnoaFactor.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::P;
using symbol_shorthand::V;

namespace {

struct WnoaFactorFixture {
  Vector Q_p1 = Vector1::Ones();
  Vector Q_p2 = Vector2::Ones();
  Vector Q_p3 = Vector3::Ones();
  Vector Q_se2 = Vector3::Ones();
  Vector Q_se3 = Vector6::Ones();
  double timestep = 0.1;

  /**** Point1 Test Variables*****/
  Point1 p0_p1{1.0};
  Vector1 v0_p1{1.0};
  // Define Second Pose with 2x the velocity (to get an error)
  Point1 p1_p1 = p0_p1 + timestep * v0_p1;
  Vector1 v1_p1 = 2.0 * v0_p1;

  /**** Point2 Test Variables*****/
  Point2 p0_p2{1.0, 2.0};
  Vector2 v0_p2{1.0, 2.0};
  // Define Second Pose with 2x the velocity (to get an error)
  Point2 p1_p2 = p0_p2 + timestep * v0_p2;
  Vector2 v1_p2 = 2.0 * v0_p2;

  /**** Point3 Test Variables*****/
  Point3 p0_p3{1.0, 2.0, 3.0};
  Vector3 v0_p3{1.0, 2.0, 3.0};
  // Define Second Pose with 2x the velocity (to get an error)
  Point3 p1_p3 = p0_p3 + timestep * v0_p3;
  Vector3 v1_p3 = 2.0 * v0_p3;

  /**** SE(2) Test Variables*****/
  Pose2 p0_se2{1.0, 0.0, 0.5};
  Vector3 v0_se2{1.0, 2.0, 0.1};
  // Define Second Pose with 2x the velocity (to get an error)
  Pose2 p1_se2 = p0_se2.expmap(timestep * v0_se2);
  Vector3 v1_se2 = 2.0 * v0_se2;

  /***** SE(3) Test Variables******/
  // Define First Pose with a general velocity
  Pose3 p0_se3 = Pose3::Expmap(Vector6(0.5, 0.0, 0.2, 1.0, 0.0, 0.0));
  Vector6 v0_se3{0.1, 0.0, 0.0, 1.0, 0.0, 2.0};
  // Define Second Pose with 2x the velocity (to get an error)
  Pose3 p1_se3 = p0_se3.expmap(timestep * v0_se3);
  Vector6 v1_se3 = 2.0 * v0_se3;

  // Define StateData structs
  StateData state_1 = StateData(P(1), V(1), 0.0);
  StateData state_2 = StateData(P(2), V(2), timestep);
};

const WnoaFactorFixture& fixture() {
  static const WnoaFactorFixture kFixture;
  return kFixture;
}

const Vector& Q_p1 = fixture().Q_p1;
const Vector& Q_p2 = fixture().Q_p2;
const Vector& Q_p3 = fixture().Q_p3;
const Vector& Q_se2 = fixture().Q_se2;
const Vector& Q_se3 = fixture().Q_se3;
const double timestep = fixture().timestep;

const Point1& p0_p1 = fixture().p0_p1;
const Vector1& v0_p1 = fixture().v0_p1;
const Point1& p1_p1 = fixture().p1_p1;
const Vector1& v1_p1 = fixture().v1_p1;

const Point2& p0_p2 = fixture().p0_p2;
const Vector2& v0_p2 = fixture().v0_p2;
const Point2& p1_p2 = fixture().p1_p2;
const Vector2& v1_p2 = fixture().v1_p2;

const Point3& p0_p3 = fixture().p0_p3;
const Vector3& v0_p3 = fixture().v0_p3;
const Point3& p1_p3 = fixture().p1_p3;
const Vector3& v1_p3 = fixture().v1_p3;

const Pose2& p0_se2 = fixture().p0_se2;
const Vector3& v0_se2 = fixture().v0_se2;
const Pose2& p1_se2 = fixture().p1_se2;
const Vector3& v1_se2 = fixture().v1_se2;

const Pose3& p0_se3 = fixture().p0_se3;
const Vector6& v0_se3 = fixture().v0_se3;
const Pose3& p1_se3 = fixture().p1_se3;
const Vector6& v1_se3 = fixture().v1_se3;

const StateData& state_1 = fixture().state_1;
const StateData& state_2 = fixture().state_2;

}  // namespace

/* ************************************************************************* */
TEST(WNOAFactor, Constructor) {
  // Create WNOA Motion Factor for Point1
  WNOAMotionFactor<Point1> factorP1(P(1), V(1), P(2), V(2), timestep, Q_p1);
  WNOAMotionFactor<Point2> factorP2(P(1), V(1), P(2), V(2), timestep, Q_p2);
  WNOAMotionFactor<Point3> factorP3(P(1), V(1), P(2), V(2), timestep, Q_p3);
  WNOAMotionFactor<Pose2> factorSE2(P(1), V(1), P(2), V(2), timestep, Q_se2);
  WNOAMotionFactor<Pose3> factorSE3(P(1), V(1), P(2), V(2), timestep, Q_se3);
}

/* ************************************************************************* */
TEST(WNOAFactor, ConstructorStateData) {
  // Create WNOA Motion Factor for Point1
  WNOAMotionFactor<Point1> factorP1(state_1, state_2, Q_p1);
  WNOAMotionFactor<Point2> factorP2(state_1, state_2, Q_p2);
  WNOAMotionFactor<Point3> factorP3(state_1, state_2, Q_p3);
  WNOAMotionFactor<Pose2> factorSE2(state_1, state_2, Q_se2);
  WNOAMotionFactor<Pose3> factorSE3(state_1, state_2, Q_se3);
}

/* *************************************************************************
 */
TEST(WNOAFactor, Equals) {
  // Create two identical factors and make sure they're equal
  WNOAMotionFactor<Pose2> factor1(P(1), V(1), P(2), V(2), timestep, Q_se2);
  WNOAMotionFactor<Pose2> factor2(P(1), V(1), P(2), V(2), timestep, Q_se2);

  CHECK(assert_equal(factor1, factor2));

  WNOAMotionFactor<Point2> factor1_point(P(1), V(1), P(2), V(2), timestep,
                                         Q_p2);
  WNOAMotionFactor<Point2> factor2_point(P(1), V(1), P(2), V(2), timestep,
                                         Q_p2);

  CHECK(assert_equal(factor1_point, factor2_point));
}

/* *************************************************************************
 */

TEST(WNOAFactor, EvalErrorP1) {
  // Define factor
  WNOAMotionFactor<Point1> factorP1(P(1), V(1), P(2), V(2), timestep, Q_p1);
  // compute error
  Vector actualError(factorP1.evaluateError(p0_p1, v0_p1, p1_p1, v1_p1));
  // expected is zero
  Vector2 expectedError;
  expectedError << Vector1(0.0), v0_p1;

  // Actual error depends on the level of accuracy we are using for the expmap
  double tol = 1e-9;

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, EvalErrorP2) {
  // Define factor
  WNOAMotionFactor<Point2> factorP2(P(1), V(1), P(2), V(2), timestep, Q_p2);
  // compute error
  Vector actualError(factorP2.evaluateError(p0_p2, v0_p2, p1_p2, v1_p2));
  // expected is zero
  Vector4 expectedError;
  expectedError << Vector2(0.0, 0.0), v0_p2;

  double tol = 1e-9;

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, EvalErrorP3) {
  // Define factor
  WNOAMotionFactor<Point3> factorP3(P(1), V(1), P(2), V(2), timestep, Q_p3);
  // compute error
  Vector actualError(factorP3.evaluateError(p0_p3, v0_p3, p1_p3, v1_p3));
  // expected is zero
  Vector6 expectedError;
  expectedError << Vector3(0.0, 0.0, 0.0), v0_p3;

  double tol = 1e-9;

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, EvalErrorSE2) {
  // Define factor
  WNOAMotionFactor<Pose2> factorSE2(P(1), V(1), P(2), V(2), timestep, Q_se2);
  // compute error
  Vector actualError(factorSE2.evaluateError(p0_se2, v0_se2, p1_se2, v1_se2));
  // expected is zero
  Vector6 expectedError;
  expectedError << Vector3(0.0, 0.0, 0.0), v0_se2;

  // Actual error depends on the level of accuracy we are using for the expmap
  double tol;
#ifdef GTSAM_SLOW_BUT_CORRECT_EXPMAP
  tol = 1e-9;
#else
  tol = 1e-3;
#endif

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, EvalErrorSE3) {
  // Define factor (keys don't matter here)
  WNOAMotionFactor<Pose3> factorSE3(P(1), V(1), P(2), V(2), timestep, Q_se3);
  // compute error
  Vector actualError(factorSE3.evaluateError(p0_se3, v0_se3, p1_se3, v1_se3));
  // expected is zero
  Vector12 expectedError;
  expectedError << Vector6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), v0_se3;

  // Actual error depends on the level of accuracy we are using for the expmap
  double tol;
#ifdef GTSAM_SLOW_BUT_CORRECT_EXPMAP
  tol = 1e-9;
#else
  tol = 1e-3;
#endif

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, EvalErrorSE3StateData) {
  // Define factor (keys don't matter here)
  WNOAMotionFactor<Pose3> factorSE3(state_1, state_2, Q_se3);
  // compute error
  Vector actualError(factorSE3.evaluateError(p0_se3, v0_se3, p1_se3, v1_se3));
  // expected is zero
  Vector12 expectedError;
  expectedError << Vector6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), v0_se3;

  // Actual error depends on the level of accuracy we are using for the expmap
  double tol;
#ifdef GTSAM_SLOW_BUT_CORRECT_EXPMAP
  tol = 1e-9;
#else
  tol = 1e-3;
#endif

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, ErrorJacobianP1) {
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WNOAMotionFactor<Point1> factorP1(P(1), V(1), P(2), V(2), timestep, Q_p1);
  auto residual = factorP1.evaluateError(p0_p1, v0_p1, p1_p1, v1_p1, J_p0, J_v0,
                                         J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorP1.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Point1, Vector1, Point1, Vector1>(
          f, p0_p1, v0_p1, p1_p1, v1_p1);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Point1, Vector1, Point1, Vector1>(
          f, p0_p1, v0_p1, p1_p1, v1_p1);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Point1, Vector1, Point1, Vector1>(
          f, p0_p1, v0_p1, p1_p1, v1_p1);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Point1, Vector1, Point1, Vector1>(
          f, p0_p1, v0_p1, p1_p1, v1_p1);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, ErrorJacobianP2) {
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WNOAMotionFactor<Point2> factorP2(P(1), V(1), P(2), V(2), timestep, Q_p2);
  auto residual = factorP2.evaluateError(p0_p2, v0_p2, p1_p2, v1_p2, J_p0, J_v0,
                                         J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorP2.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Point2, Vector2, Point2, Vector2>(
          f, p0_p2, v0_p2, p1_p2, v1_p2);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Point2, Vector2, Point2, Vector2>(
          f, p0_p2, v0_p2, p1_p2, v1_p2);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Point2, Vector2, Point2, Vector2>(
          f, p0_p2, v0_p2, p1_p2, v1_p2);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Point2, Vector2, Point2, Vector2>(
          f, p0_p2, v0_p2, p1_p2, v1_p2);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, ErrorJacobianP3) {
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WNOAMotionFactor<Point3> factorP3(P(1), V(1), P(2), V(2), timestep, Q_p3);
  auto residual = factorP3.evaluateError(p0_p3, v0_p3, p1_p3, v1_p3, J_p0, J_v0,
                                         J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorP3.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Point3, Vector3, Point3, Vector3>(
          f, p0_p3, v0_p3, p1_p3, v1_p3);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Point3, Vector3, Point3, Vector3>(
          f, p0_p3, v0_p3, p1_p3, v1_p3);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Point3, Vector3, Point3, Vector3>(
          f, p0_p3, v0_p3, p1_p3, v1_p3);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Point3, Vector3, Point3, Vector3>(
          f, p0_p3, v0_p3, p1_p3, v1_p3);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, ErrorJacobianSE2) {
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WNOAMotionFactor<Pose2> factorSE2(P(1), V(1), P(2), V(2), timestep, Q_se2);
  auto residual = factorSE2.evaluateError(p0_se2, v0_se2, p1_se2, v1_se2, J_p0,
                                          J_v0, J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorSE2.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, p0_se2, v0_se2, p1_se2, v1_se2);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, p0_se2, v0_se2, p1_se2, v1_se2);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, p0_se2, v0_se2, p1_se2, v1_se2);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, p0_se2, v0_se2, p1_se2, v1_se2);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, ErrorJacobianSE3) {
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WNOAMotionFactor<Pose3> factorSE3(P(1), V(1), P(2), V(2), timestep, Q_se3);
  auto residual = factorSE3.evaluateError(p0_se3, v0_se3, p1_se3, v1_se3, J_p0,
                                          J_v0, J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorSE3.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p1_se3, v1_se3);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p1_se3, v1_se3);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p1_se3, v1_se3);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p1_se3, v1_se3);

  double tol = 1e-4;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

/* *************************************************************************
 */

TEST(WNOAFactor, OptimizeP1) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point1>>(P(0), p0_p1);
  graph.emplace_shared<NonlinearEquality<Vector1>>(V(0), v0_p1);

  // add WNOA factor
  graph.add(WNOAMotionFactor<Point1>(P(0), V(0), P(1), V(1), timestep, Q_p1));
  // Set up initial guess
  Values values;
  values.insert(P(0), p0_p1);
  values.insert(V(0), v0_p1);
  values.insert(P(1), p1_p1);
  values.insert(V(1), v0_p1);

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // We expect the initial to be zero because config is the ground truth
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Complete solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* *************************************************************************
 */

TEST(WNOAFactor, OptimizeP2) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point2>>(P(0), p0_p2);
  graph.emplace_shared<NonlinearEquality<Vector2>>(V(0), v0_p2);

  // add WNOA factor
  graph.add(WNOAMotionFactor<Point2>(P(0), V(0), P(1), V(1), timestep, Q_p2));
  // Set up initial guess
  Values values;
  values.insert(P(0), p0_p2);
  values.insert(V(0), v0_p2);
  values.insert(P(1), p1_p2);
  values.insert(V(1), v0_p2);

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // We expect the initial to be zero because config is the ground truth
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Complete solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* *************************************************************************
 */

TEST(WNOAFactor, OptimizeP3) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point3>>(P(0), p0_p3);
  graph.emplace_shared<NonlinearEquality<Vector3>>(V(0), v0_p3);

  // add WNOA factor
  graph.add(WNOAMotionFactor<Point3>(P(0), V(0), P(1), V(1), timestep, Q_p3));
  // Set up initial guess
  Values values;
  values.insert(P(0), p0_p3);
  values.insert(V(0), v0_p3);
  values.insert(P(1), p1_p3);
  values.insert(V(1), v0_p3);

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // We expect the initial to be zero because config is the ground truth
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Complete solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* *************************************************************************
 */

TEST(WNOAFactor, OptimizeSE2) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Pose2>>(P(0), p0_se2);
  graph.emplace_shared<NonlinearEquality<Vector3>>(V(0), v0_se2);

  // add WNOA factor
  graph.add(WNOAMotionFactor<Pose2>(P(0), V(0), P(1), V(1), timestep, Q_se2));
  // Set up initial guess
  Values values;
  values.insert(P(0), p0_se2);
  values.insert(V(0), v0_se2);
  values.insert(P(1), p1_se2);
  values.insert(V(1), v0_se2);

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // We expect the initial to be zero because config is the ground truth
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Complete solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* ************************************************************************* */

TEST(WNOAFactor, OptimizeSE3) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Pose3>>(P(0), p0_se3);
  graph.emplace_shared<NonlinearEquality<Vector6>>(V(0), v0_se3);

  // add WNOA factor
  graph.add(WNOAMotionFactor<Pose3>(P(0), V(0), P(1), V(1), timestep, Q_se3));
  // Set up initial guess
  Values values;
  values.insert(V(0), v0_se3);
  values.insert(P(0), p0_se3);
  values.insert(P(1), p1_se3);
  values.insert(V(1), v0_se3);

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // We expect the initial to be zero because config is the ground truth
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Complete solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* ************************************************************************* */

TEST(WNOAFactor, OptimizeP3Pert) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point3>>(P(0), p0_p3);
  graph.emplace_shared<NonlinearEquality<Vector3>>(V(0), v0_p3);

  // add WNOA factor
  graph.add(WNOAMotionFactor<Point3>(P(0), V(0), P(1), V(1), timestep, Q_p3));
  // Set up initial guess
  Values values;
  values.insert(V(0), v0_p3);
  values.insert(P(0), p0_p3);

  // Perturb the pose and velocity
  values.insert(V(1), v0_p3 + Vector3(1.0, 1.0, 1.0) * 0.5);
  values.insert(P(1), p1_p3 + Vector3(1.0, 1.0, 1.0) * 0.5);

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // Check that we converge to solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* ************************************************************************* */

TEST(WNOAFactor, OptimizeSE3Pert) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Pose3>>(P(0), p0_se3);
  graph.emplace_shared<NonlinearEquality<Vector6>>(V(0), v0_se3);

  // add WNOA factor
  graph.add(WNOAMotionFactor<Pose3>(P(0), V(0), P(1), V(1), timestep, Q_se3));
  // Set up initial guess
  Values values;
  values.insert(V(0), v0_se3);
  values.insert(P(0), p0_se3);

  // Perturb the pose and velocity
  values.insert(V(1), v0_se3 + Vector6(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) * 0.1);
  values.insert(P(1),
                p1_se3.expmap(Vector6(0.001, 0.001, 0.001, 0.1, 0.1, 0.1)));

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // Check that we converge to solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
