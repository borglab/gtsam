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
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/NonlinearEquality.h>
#include <gtsam/geometry/Point1.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/WnoaFactor.h>
#include <gtsam/nonlinear/WnoaStateData.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::P;
using symbol_shorthand::V;

/* ************************************************************************* */
namespace construction {

constexpr double kTimestep = 0.1;

// Construct factors from keys for every supported state type.
TEST(WnoaFactor, Constructor) {
  WnoaMotionFactor<Point1> factorP1(P(1), V(1), P(2), V(2), kTimestep,
                                    Vector1::Ones());
  WnoaMotionFactor<Point2> factorP2(P(1), V(1), P(2), V(2), kTimestep,
                                    Vector2::Ones());
  WnoaMotionFactor<Point3> factorP3(P(1), V(1), P(2), V(2), kTimestep,
                                    Vector3::Ones());
  WnoaMotionFactor<Pose2> factorSE2(P(1), V(1), P(2), V(2), kTimestep,
                                    Vector3::Ones());
  WnoaMotionFactor<Pose3> factorSE3(P(1), V(1), P(2), V(2), kTimestep,
                                    Vector6::Ones());
}

// Construct factors from timestamped state keys.
TEST(WnoaFactor, ConstructorStateData) {
  const StateData state0(P(1), V(1), 0.0);
  const StateData state1(P(2), V(2), kTimestep);
  WnoaMotionFactor<Point1> factorP1(state0, state1, Vector1::Ones());
  WnoaMotionFactor<Point2> factorP2(state0, state1, Vector2::Ones());
  WnoaMotionFactor<Point3> factorP3(state0, state1, Vector3::Ones());
  WnoaMotionFactor<Pose2> factorSE2(state0, state1, Vector3::Ones());
  WnoaMotionFactor<Pose3> factorSE3(state0, state1, Vector6::Ones());
}

// Factors with identical keys and parameters compare equal.
TEST(WnoaFactor, Equals) {
  // Create two identical factors and make sure they're equal
  WnoaMotionFactor<Pose2> factor1(P(1), V(1), P(2), V(2), kTimestep,
                                  Vector3::Ones());
  WnoaMotionFactor<Pose2> factor2(P(1), V(1), P(2), V(2), kTimestep,
                                  Vector3::Ones());

  CHECK(assert_equal(factor1, factor2));

  WnoaMotionFactor<Point2> factor1_point(P(1), V(1), P(2), V(2), kTimestep,
                                         Vector2::Ones());
  WnoaMotionFactor<Point2> factor2_point(P(1), V(1), P(2), V(2), kTimestep,
                                         Vector2::Ones());

  CHECK(assert_equal(factor1_point, factor2_point));
}

}  // namespace construction
/* ************************************************************************* */
namespace point1 {

constexpr double kTimestep = 0.1;

struct Fixture {
  Vector1 psd = Vector1::Ones();
  Point1 pose0{1.0};
  Vector1 velocity0{1.0};
  Point1 pose1 = pose0 + kTimestep * velocity0;
  // Double the second velocity to produce a nonzero residual.
  Vector1 velocity1 = 2.0 * velocity0;
};

// Verify the residual for a trajectory with a velocity change.
TEST(WnoaFactor, EvalErrorP1) {
  const Fixture data;
  // Define factor
  WnoaMotionFactor<Point1> factorP1(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  // compute error
  Vector actualError(factorP1.evaluateError(data.pose0, data.velocity0,
                                            data.pose1, data.velocity1));
  // expected is zero
  Vector2 expectedError(0.0, data.velocity0(0));

  // Actual error depends on the level of accuracy we are using for the expmap
  double tol = 1e-9;

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

// Compare all four analytic Jacobians with numerical derivatives.
TEST(WnoaFactor, ErrorJacobianP1) {
  const Fixture data;
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WnoaMotionFactor<Point1> factorP1(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  auto residual =
      factorP1.evaluateError(data.pose0, data.velocity0, data.pose1,
                             data.velocity1, J_p0, J_v0, J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorP1.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Point1, Vector1, Point1, Vector1>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Point1, Vector1, Point1, Vector1>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Point1, Vector1, Point1, Vector1>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Point1, Vector1, Point1, Vector1>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

// Verify optimization preserves a zero-error trajectory.
TEST(WnoaFactor, OptimizeP1) {
  const Fixture data;
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point1>>(P(0), data.pose0);
  graph.emplace_shared<NonlinearEquality<Vector1>>(V(0), data.velocity0);

  // add WNOA factor
  graph.add(
      WnoaMotionFactor<Point1>(P(0), V(0), P(1), V(1), kTimestep, data.psd));
  // Set up initial guess
  Values values;
  values.insert(P(0), data.pose0);
  values.insert(V(0), data.velocity0);
  values.insert(P(1), data.pose1);
  values.insert(V(1), data.velocity0);

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

}  // namespace point1
/* ************************************************************************* */
namespace point2 {

constexpr double kTimestep = 0.1;

struct Fixture {
  Vector2 psd = Vector2::Ones();
  Point2 pose0{1.0, 2.0};
  Vector2 velocity0{1.0, 2.0};
  Point2 pose1 = pose0 + kTimestep * velocity0;
  // Double the second velocity to produce a nonzero residual.
  Vector2 velocity1 = 2.0 * velocity0;
};

// Verify the residual for a trajectory with a velocity change.
TEST(WnoaFactor, EvalErrorP2) {
  const Fixture data;
  // Define factor
  WnoaMotionFactor<Point2> factorP2(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  // compute error
  Vector actualError(factorP2.evaluateError(data.pose0, data.velocity0,
                                            data.pose1, data.velocity1));
  // expected is zero
  Vector4 expectedError;
  expectedError << Vector2(0.0, 0.0), data.velocity0;

  double tol = 1e-9;

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

// Compare all four analytic Jacobians with numerical derivatives.
TEST(WnoaFactor, ErrorJacobianP2) {
  const Fixture data;
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WnoaMotionFactor<Point2> factorP2(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  auto residual =
      factorP2.evaluateError(data.pose0, data.velocity0, data.pose1,
                             data.velocity1, J_p0, J_v0, J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorP2.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Point2, Vector2, Point2, Vector2>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Point2, Vector2, Point2, Vector2>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Point2, Vector2, Point2, Vector2>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Point2, Vector2, Point2, Vector2>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

// Verify optimization preserves a zero-error trajectory.
TEST(WnoaFactor, OptimizeP2) {
  const Fixture data;
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point2>>(P(0), data.pose0);
  graph.emplace_shared<NonlinearEquality<Vector2>>(V(0), data.velocity0);

  // add WNOA factor
  graph.add(
      WnoaMotionFactor<Point2>(P(0), V(0), P(1), V(1), kTimestep, data.psd));
  // Set up initial guess
  Values values;
  values.insert(P(0), data.pose0);
  values.insert(V(0), data.velocity0);
  values.insert(P(1), data.pose1);
  values.insert(V(1), data.velocity0);

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

}  // namespace point2
/* ************************************************************************* */
namespace point3 {

constexpr double kTimestep = 0.1;

struct Fixture {
  Vector3 psd = Vector3::Ones();
  Point3 pose0{1.0, 2.0, 3.0};
  Vector3 velocity0{1.0, 2.0, 3.0};
  Point3 pose1 = pose0 + kTimestep * velocity0;
  // Double the second velocity to produce a nonzero residual.
  Vector3 velocity1 = 2.0 * velocity0;
};

// Verify the residual for a trajectory with a velocity change.
TEST(WnoaFactor, EvalErrorP3) {
  const Fixture data;
  // Define factor
  WnoaMotionFactor<Point3> factorP3(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  // compute error
  Vector actualError(factorP3.evaluateError(data.pose0, data.velocity0,
                                            data.pose1, data.velocity1));
  // expected is zero
  Vector6 expectedError;
  expectedError << Vector3(0.0, 0.0, 0.0), data.velocity0;

  double tol = 1e-9;

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, tol));
}

// Compare all four analytic Jacobians with numerical derivatives.
TEST(WnoaFactor, ErrorJacobianP3) {
  const Fixture data;
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WnoaMotionFactor<Point3> factorP3(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  auto residual =
      factorP3.evaluateError(data.pose0, data.velocity0, data.pose1,
                             data.velocity1, J_p0, J_v0, J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorP3.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Point3, Vector3, Point3, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Point3, Vector3, Point3, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Point3, Vector3, Point3, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Point3, Vector3, Point3, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

// Verify optimization preserves a zero-error trajectory.
TEST(WnoaFactor, OptimizeP3) {
  const Fixture data;
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point3>>(P(0), data.pose0);
  graph.emplace_shared<NonlinearEquality<Vector3>>(V(0), data.velocity0);

  // add WNOA factor
  graph.add(
      WnoaMotionFactor<Point3>(P(0), V(0), P(1), V(1), kTimestep, data.psd));
  // Set up initial guess
  Values values;
  values.insert(P(0), data.pose0);
  values.insert(V(0), data.velocity0);
  values.insert(P(1), data.pose1);
  values.insert(V(1), data.velocity0);

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

// Verify optimization recovers the trajectory from a perturbed start.
TEST(WnoaFactor, OptimizeP3Pert) {
  const Fixture data;
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Point3>>(P(0), data.pose0);
  graph.emplace_shared<NonlinearEquality<Vector3>>(V(0), data.velocity0);

  // add WNOA factor
  graph.add(
      WnoaMotionFactor<Point3>(P(0), V(0), P(1), V(1), kTimestep, data.psd));
  // Set up initial guess
  Values values;
  values.insert(V(0), data.velocity0);
  values.insert(P(0), data.pose0);

  // Perturb the pose and velocity
  values.insert(V(1), data.velocity0 + Vector3(1.0, 1.0, 1.0) * 0.5);
  values.insert(P(1), data.pose1 + Vector3(1.0, 1.0, 1.0) * 0.5);

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // Check that we converge to solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

}  // namespace point3
/* ************************************************************************* */
namespace se2 {

constexpr double kTimestep = 0.1;

struct Fixture {
  Vector3 psd = Vector3::Ones();
  Pose2 pose0{1.0, 0.0, 0.5};
  Vector3 velocity0{1.0, 2.0, 0.1};
  Pose2 pose1 = pose0.expmap(kTimestep * velocity0);
  // Double the second velocity to produce a nonzero residual.
  Vector3 velocity1 = 2.0 * velocity0;
};

// Verify the residual for a trajectory with a velocity change.
TEST(WnoaFactor, EvalErrorSE2) {
  const Fixture data;
  // Define factor
  WnoaMotionFactor<Pose2> factorSE2(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  // compute error
  Vector actualError(factorSE2.evaluateError(data.pose0, data.velocity0,
                                             data.pose1, data.velocity1));
  // expected is zero
  Vector6 expectedError;
  expectedError << Vector3(0.0, 0.0, 0.0), data.velocity0;

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

// Compare all four analytic Jacobians with numerical derivatives.
TEST(WnoaFactor, ErrorJacobianSE2) {
  const Fixture data;
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WnoaMotionFactor<Pose2> factorSE2(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  auto residual =
      factorSE2.evaluateError(data.pose0, data.velocity0, data.pose1,
                              data.velocity1, J_p0, J_v0, J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorSE2.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Pose2, Vector3, Pose2, Vector3>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);

  double tol = 1e-6;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

// Verify optimization preserves a zero-error trajectory.
TEST(WnoaFactor, OptimizeSE2) {
  const Fixture data;
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Pose2>>(P(0), data.pose0);
  graph.emplace_shared<NonlinearEquality<Vector3>>(V(0), data.velocity0);

  // add WNOA factor
  graph.add(
      WnoaMotionFactor<Pose2>(P(0), V(0), P(1), V(1), kTimestep, data.psd));
  // Set up initial guess
  Values values;
  values.insert(P(0), data.pose0);
  values.insert(V(0), data.velocity0);
  values.insert(P(1), data.pose1);
  values.insert(V(1), data.velocity0);

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

}  // namespace se2
/* ************************************************************************* */
namespace se3 {

constexpr double kTimestep = 0.1;

struct Fixture {
  Vector6 psd = Vector6::Ones();
  Pose3 pose0 = Pose3::Expmap(Vector6(0.5, 0.0, 0.2, 1.0, 0.0, 0.0));
  Vector6 velocity0{0.1, 0.0, 0.0, 1.0, 0.0, 2.0};
  Pose3 pose1 = pose0.expmap(kTimestep * velocity0);
  // Double the second velocity to produce a nonzero residual.
  Vector6 velocity1 = 2.0 * velocity0;
};

// Verify the residual for a trajectory with a velocity change.
TEST(WnoaFactor, EvalErrorSE3) {
  const Fixture data;
  // Define factor (keys don't matter here)
  WnoaMotionFactor<Pose3> factorSE3(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  // compute error
  Vector actualError(factorSE3.evaluateError(data.pose0, data.velocity0,
                                             data.pose1, data.velocity1));
  // expected is zero
  Vector12 expectedError;
  expectedError << Vector6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), data.velocity0;

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

// Verify the residual when constructing from StateData.
TEST(WnoaFactor, EvalErrorSE3StateData) {
  const Fixture data;
  const StateData state0(P(1), V(1), 0.0);
  const StateData state1(P(2), V(2), kTimestep);
  // Define factor (keys don't matter here)
  WnoaMotionFactor<Pose3> factorSE3(state0, state1, data.psd);
  // compute error
  Vector actualError(factorSE3.evaluateError(data.pose0, data.velocity0,
                                             data.pose1, data.velocity1));
  // expected is zero
  Vector12 expectedError;
  expectedError << Vector6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), data.velocity0;

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

// Compare all four analytic Jacobians with numerical derivatives.
TEST(WnoaFactor, ErrorJacobianSE3) {
  const Fixture data;
  // Get analytical derivatives
  Matrix J_p0, J_v0, J_p1, J_v1;
  WnoaMotionFactor<Pose3> factorSE3(P(1), V(1), P(2), V(2), kTimestep,
                                    data.psd);
  auto residual =
      factorSE3.evaluateError(data.pose0, data.velocity0, data.pose1,
                              data.velocity1, J_p0, J_v0, J_p1, J_v1);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p1, auto& v1) {
    return factorSE3.evaluateError(p0, v0, p1, v1);
  };

  // Compute numerical derivatives
  Matrix J_p0_num =
      numericalDerivative41<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_p1_num =
      numericalDerivative43<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);
  Matrix J_v1_num =
      numericalDerivative44<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, data.pose0, data.velocity0, data.pose1, data.velocity1);

  double tol = 1e-4;
  EXPECT(assert_equal(J_p0, J_p0_num, tol));
  EXPECT(assert_equal(J_v0, J_v0_num, tol));
  EXPECT(assert_equal(J_p1, J_p1_num, tol));
  EXPECT(assert_equal(J_v1, J_v1_num, tol));
}

// Verify optimization preserves a zero-error trajectory.
TEST(WnoaFactor, OptimizeSE3) {
  const Fixture data;
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Pose3>>(P(0), data.pose0);
  graph.emplace_shared<NonlinearEquality<Vector6>>(V(0), data.velocity0);

  // add WNOA factor
  graph.add(
      WnoaMotionFactor<Pose3>(P(0), V(0), P(1), V(1), kTimestep, data.psd));
  // Set up initial guess
  Values values;
  values.insert(V(0), data.velocity0);
  values.insert(P(0), data.pose0);
  values.insert(P(1), data.pose1);
  values.insert(V(1), data.velocity0);

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

// Verify optimization recovers the trajectory from a perturbed start.
TEST(WnoaFactor, OptimizeSE3Pert) {
  const Fixture data;
  // Create a factor graph
  NonlinearFactorGraph graph;
  // Lock the first pose
  graph.emplace_shared<NonlinearEquality<Pose3>>(P(0), data.pose0);
  graph.emplace_shared<NonlinearEquality<Vector6>>(V(0), data.velocity0);

  // add WNOA factor
  graph.add(
      WnoaMotionFactor<Pose3>(P(0), V(0), P(1), V(1), kTimestep, data.psd));
  // Set up initial guess
  Values values;
  values.insert(V(0), data.velocity0);
  values.insert(P(0), data.pose0);

  // Perturb the pose and velocity
  values.insert(V(1),
                data.velocity0 + Vector6(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) * 0.1);
  values.insert(P(1),
                data.pose1.expmap(Vector6(0.001, 0.001, 0.001, 0.1, 0.1, 0.1)));

  // Set up optimizer
  GaussNewtonOptimizer optimizer(graph, values);

  // Check that we converge to solution
  optimizer.optimize();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

}  // namespace se3
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
