/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testReferenceFrameFactor.cpp
 * @author Alex Cunningham
 */

#include <iostream>

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/slam/ReferenceFrameFactor.h>

using namespace std;
using namespace boost;
using namespace std::placeholders;
using namespace gtsam;

typedef gtsam::ReferenceFrameFactor<gtsam::Point2, gtsam::Pose2> PointReferenceFrameFactor;
typedef gtsam::ReferenceFrameFactor<gtsam::Pose2, gtsam::Pose2> PoseReferenceFrameFactor;

Key lA1 = symbol_shorthand::L(1), lA2 = symbol_shorthand::L(2), lB1 = symbol_shorthand::L(11), lB2 = symbol_shorthand::L(12);
Key tA1 = symbol_shorthand::T(1), tB1 = symbol_shorthand::T(2);

/* ************************************************************************* */
TEST( ReferenceFrameFactor, equals ) {
  PointReferenceFrameFactor
    c1(lB1, tA1, lA1),
    c2(lB1, tA1, lA1),
    c3(lB1, tA1, lA2);

  EXPECT(assert_equal(c1, c1));
  EXPECT(assert_equal(c1, c2));
  EXPECT(!c1.equals(c3));
}

/* ************************************************************************* */
Vector evaluateError_(const PointReferenceFrameFactor& c,
    const Point2& global, const Pose2& trans, const Point2& local) {
  return Vector(c.evaluateError(global, trans, local));
}
TEST( ReferenceFrameFactor, jacobians ) {

  // from examples below
  Point2 local(2.0, 3.0), global(-1.0, 2.0);
  Pose2 trans(1.5, 2.5, 0.3);

  PointReferenceFrameFactor tc(lA1, tA1, lB1);
  Matrix actualDT, actualDL, actualDF;
  tc.evaluateError(global, trans, local, actualDF, actualDT, actualDL);

  Matrix numericalDT, numericalDL, numericalDF;
  numericalDF = numericalDerivative31<Vector,Point2,Pose2,Point2>(
      std::bind(evaluateError_, tc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      global, trans, local, 1e-5);
  numericalDT = numericalDerivative32<Vector,Point2,Pose2,Point2>(
      std::bind(evaluateError_, tc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      global, trans, local, 1e-5);
  numericalDL = numericalDerivative33<Vector,Point2,Pose2,Point2>(
      std::bind(evaluateError_, tc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      global, trans, local, 1e-5);

  EXPECT(assert_equal(numericalDF, actualDF));
  EXPECT(assert_equal(numericalDL, actualDL));
  EXPECT(assert_equal(numericalDT, actualDT));
}

/* ************************************************************************* */
TEST( ReferenceFrameFactor, jacobians_zero ) {

  // get values that are ideal
  Pose2 trans(2.0, 3.0, 0.0);
  Point2 global(5.0, 6.0);
  Point2 local = trans.transformFrom(global);

  PointReferenceFrameFactor tc(lA1, tA1, lB1);
  Vector actCost = tc.evaluateError(global, trans, local),
      expCost = Z_2x1;
  EXPECT(assert_equal(expCost, actCost, 1e-5));

  Matrix actualDT, actualDL, actualDF;
  tc.evaluateError(global, trans, local, actualDF, actualDT, actualDL);

  Matrix numericalDT, numericalDL, numericalDF;
  numericalDF = numericalDerivative31<Vector,Point2,Pose2,Point2>(
      std::bind(evaluateError_, tc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      global, trans, local, 1e-5);
  numericalDT = numericalDerivative32<Vector,Point2,Pose2,Point2>(
      std::bind(evaluateError_, tc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      global, trans, local, 1e-5);
  numericalDL = numericalDerivative33<Vector,Point2,Pose2,Point2>(
      std::bind(evaluateError_, tc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      global, trans, local, 1e-5);

  EXPECT(assert_equal(numericalDF, actualDF));
  EXPECT(assert_equal(numericalDL, actualDL));
  EXPECT(assert_equal(numericalDT, actualDT));
}

/* ************************************************************************* */
TEST( ReferenceFrameFactor, converge_trans ) {

  // initial points
  Point2 local1(2.0, 2.0), local2(4.0, 5.0),
      global1(-1.0, 5.0), global2(2.0, 3.0);
  Pose2 transIdeal(7.0, 3.0, M_PI/2);

  // verify direction
  EXPECT(assert_equal(local1, transIdeal.transformFrom(global1)));
  EXPECT(assert_equal(local2, transIdeal.transformFrom(global2)));

  // choose transform
  //  Pose2 trans = transIdeal; // ideal - works
  //  Pose2 trans = transIdeal * Pose2(0.1, 1.0, 0.00);  // translation - works
  //  Pose2 trans = transIdeal * Pose2(10.1, 1.0, 0.00);  // large translation - works
  //  Pose2 trans = transIdeal * Pose2(0.0, 0.0, 0.1);   // small rotation - works
  Pose2 trans = transIdeal * Pose2(-200.0, 100.0, 1.3); // combined - works
  //  Pose2 trans = transIdeal * Pose2(-200.0, 100.0, 2.0); // beyond pi/2 - fails

  NonlinearFactorGraph graph;
  graph.emplace_shared<PointReferenceFrameFactor>(lB1, tA1, lA1);
  graph.emplace_shared<PointReferenceFrameFactor>(lB2, tA1, lA2);

  // hard constraints on points
  double error_gain = 1000.0;
  graph.emplace_shared<NonlinearEquality<gtsam::Point2> >(lA1, local1, error_gain);
  graph.emplace_shared<NonlinearEquality<gtsam::Point2> >(lA2, local2, error_gain);
  graph.emplace_shared<NonlinearEquality<gtsam::Point2> >(lB1, global1, error_gain);
  graph.emplace_shared<NonlinearEquality<gtsam::Point2> >(lB2, global2, error_gain);

  // create initial estimate
  Values init;
  init.insert(lA1, local1);
  init.insert(lA2, local2);
  init.insert(lB1, global1);
  init.insert(lB2, global2);
  init.insert(tA1, trans);

  // optimize
  LevenbergMarquardtOptimizer solver(graph, init);
  Values actual = solver.optimize();

  Values expected;
  expected.insert(lA1, local1);
  expected.insert(lA2, local2);
  expected.insert(lB1, global1);
  expected.insert(lB2, global2);
  expected.insert(tA1, transIdeal);

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST( ReferenceFrameFactor, converge_local ) {

  // initial points
  Point2 global(-1.0, 2.0);
  //  Pose2 trans(1.5, 2.5, 0.3); // original
  //  Pose2 trans(1.5, 2.5, 1.0); // larger rotation
  Pose2 trans(1.5, 2.5, 3.1); // significant rotation

  Point2 idealLocal = trans.transformFrom(global);

  // perturb the initial estimate
  //  Point2 local = idealLocal; // Ideal case - works
  //  Point2 local = idealLocal + Point2(1.0, 0.0); // works
  Point2 local = idealLocal + Point2(-10.0, 10.0); // works

  NonlinearFactorGraph graph;
  double error_gain = 1000.0;
  graph.emplace_shared<PointReferenceFrameFactor>(lB1, tA1, lA1);
  graph.emplace_shared<NonlinearEquality<gtsam::Point2> >(lB1, global, error_gain);
  graph.emplace_shared<NonlinearEquality<gtsam::Pose2> >(tA1, trans, error_gain);

  // create initial estimate
  Values init;
  init.insert(lA1, local);
  init.insert(lB1, global);
  init.insert(tA1, trans);

  // optimize
  LevenbergMarquardtOptimizer solver(graph, init);
  Values actual = solver.optimize();

  CHECK(actual.exists(lA1));
  EXPECT(assert_equal(idealLocal, actual.at<Point2>(lA1), 1e-5));
}

/* ************************************************************************* */
TEST( ReferenceFrameFactor, converge_global ) {

  // initial points
  Point2 local(2.0, 3.0);
  //  Pose2 trans(1.5, 2.5, 0.3); // original
  //  Pose2 trans(1.5, 2.5, 1.0); // larger rotation
  Pose2 trans(1.5, 2.5, 3.1); // significant rotation

  Point2 idealForeign = trans.inverse().transformFrom(local);

  // perturb the initial estimate
  //  Point2 global = idealForeign; // Ideal - works
  //  Point2 global = idealForeign + Point2(1.0, 0.0); // simple - works
  Point2 global = idealForeign + Point2(10.0, -10.0); // larger - works

  NonlinearFactorGraph graph;
  double error_gain = 1000.0;
  graph.emplace_shared<PointReferenceFrameFactor>(lB1, tA1, lA1);
  graph.emplace_shared<NonlinearEquality<gtsam::Point2> >(lA1, local, error_gain);
  graph.emplace_shared<NonlinearEquality<gtsam::Pose2> >(tA1, trans, error_gain);

  // create initial estimate
  Values init;
  init.insert(lA1, local);
  init.insert(lB1, global);
  init.insert(tA1, trans);

  // optimize
  LevenbergMarquardtOptimizer solver(graph, init);
  Values actual = solver.optimize();

  // verify
  CHECK(actual.exists(lB1));
  EXPECT(assert_equal(idealForeign, actual.at<Point2>(lB1), 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


