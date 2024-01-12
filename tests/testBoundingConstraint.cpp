/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBoundingConstraint.cpp
 * @brief test of nonlinear inequality constraints on scalar bounds
 * @author Alex Cunningham
 */

#include <tests/simulated2DConstraints.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <CppUnitLite/TestHarness.h>

namespace iq2D = simulated2D::inequality_constraints;
using namespace std;
using namespace gtsam;

static const double tol = 1e-5;

SharedDiagonal soft_model2 = noiseModel::Unit::Create(2);
SharedDiagonal soft_model2_alt = noiseModel::Isotropic::Sigma(2, 0.1);
SharedDiagonal hard_model1 = noiseModel::Constrained::All(1);

// some simple inequality constraints
gtsam::Key key = 1;
double mu = 10.0;
// greater than
iq2D::PoseXInequality constraint1(key, 1.0, true, mu);
iq2D::PoseYInequality constraint2(key, 2.0, true, mu);

// less than
iq2D::PoseXInequality constraint3(key, 1.0, false, mu);
iq2D::PoseYInequality constraint4(key, 2.0, false, mu);

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_basics_inactive1 ) {
  Point2 pt1(2.0, 3.0);
  Values config;
  config.insert(key, pt1);
  EXPECT(!constraint1.active(config));
  EXPECT(!constraint2.active(config));
  EXPECT_DOUBLES_EQUAL(1.0, constraint1.threshold(), tol);
  EXPECT_DOUBLES_EQUAL(2.0, constraint2.threshold(), tol);
  EXPECT(constraint1.isGreaterThan());
  EXPECT(constraint2.isGreaterThan());
  EXPECT(assert_equal(I_1x1, constraint1.evaluateError(pt1), tol));
  EXPECT(assert_equal(I_1x1, constraint2.evaluateError(pt1), tol));
  EXPECT(assert_equal(Z_1x1, constraint1.unwhitenedError(config), tol));
  EXPECT(assert_equal(Z_1x1, constraint2.unwhitenedError(config), tol));
  EXPECT_DOUBLES_EQUAL(0.0, constraint1.error(config), tol);
  EXPECT_DOUBLES_EQUAL(0.0, constraint2.error(config), tol);
}

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_basics_inactive2 ) {
  Point2 pt2(-2.0, -3.0);
  Values config;
  config.insert(key, pt2);
  EXPECT(!constraint3.active(config));
  EXPECT(!constraint4.active(config));
  EXPECT_DOUBLES_EQUAL(1.0, constraint3.threshold(), tol);
  EXPECT_DOUBLES_EQUAL(2.0, constraint4.threshold(), tol);
  EXPECT(!constraint3.isGreaterThan());
  EXPECT(!constraint4.isGreaterThan());
  EXPECT(assert_equal(Vector::Constant(1, 3.0), constraint3.evaluateError(pt2), tol));
  EXPECT(assert_equal(Vector::Constant(1, 5.0), constraint4.evaluateError(pt2), tol));
  EXPECT(assert_equal(Z_1x1, constraint3.unwhitenedError(config), tol));
  EXPECT(assert_equal(Z_1x1, constraint4.unwhitenedError(config), tol));
  EXPECT_DOUBLES_EQUAL(0.0, constraint3.error(config), tol);
  EXPECT_DOUBLES_EQUAL(0.0, constraint4.error(config), tol);
}

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_basics_active1 ) {
  Point2 pt2(-2.0, -3.0);
  Values config;
  config.insert(key, pt2);
  EXPECT(constraint1.active(config));
  EXPECT(constraint2.active(config));
  EXPECT(assert_equal(Vector::Constant(1,-3.0), constraint1.evaluateError(pt2), tol));
  EXPECT(assert_equal(Vector::Constant(1,-5.0), constraint2.evaluateError(pt2), tol));
  EXPECT(assert_equal(Vector::Constant(1,-3.0), constraint1.unwhitenedError(config), tol));
  EXPECT(assert_equal(Vector::Constant(1,-5.0), constraint2.unwhitenedError(config), tol));
  EXPECT_DOUBLES_EQUAL(45.0, constraint1.error(config), tol);
  EXPECT_DOUBLES_EQUAL(125.0, constraint2.error(config), tol);
}

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_basics_active2 ) {
  Point2 pt1(2.0, 3.0);
  Values config;
  config.insert(key, pt1);
  EXPECT(constraint3.active(config));
  EXPECT(constraint4.active(config));
  EXPECT(assert_equal(-1.0 * I_1x1, constraint3.evaluateError(pt1), tol));
  EXPECT(assert_equal(-1.0 * I_1x1, constraint4.evaluateError(pt1), tol));
  EXPECT(assert_equal(-1.0 * I_1x1, constraint3.unwhitenedError(config), tol));
  EXPECT(assert_equal(-1.0 * I_1x1, constraint4.unwhitenedError(config), tol));
  EXPECT_DOUBLES_EQUAL(5.0, constraint3.error(config), tol);
  EXPECT_DOUBLES_EQUAL(5.0, constraint4.error(config), tol);
}

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_linearization_inactive) {
  Point2 pt1(2.0, 3.0);
  Values config1;
  config1.insert(key, pt1);
  GaussianFactor::shared_ptr actual1 = constraint1.linearize(config1);
  GaussianFactor::shared_ptr actual2 = constraint2.linearize(config1);
  EXPECT(!actual1);
  EXPECT(!actual2);
}

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_linearization_active) {
  Point2 pt2(-2.0, -3.0);
  Values config2;
  config2.insert(key, pt2);
  GaussianFactor::shared_ptr actual1 = constraint1.linearize(config2);
  GaussianFactor::shared_ptr actual2 = constraint2.linearize(config2);
  JacobianFactor expected1(key, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector::Constant(1, 3.0), hard_model1);
  JacobianFactor expected2(key, (Matrix(1, 2) << 0.0, 1.0).finished(), Vector::Constant(1, 5.0), hard_model1);
  EXPECT(assert_equal((const GaussianFactor&)expected1, *actual1, tol));
  EXPECT(assert_equal((const GaussianFactor&)expected2, *actual2, tol));
}

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_simple_optimization1) {
  // create a single-node graph with a soft and hard constraint to
  // ensure that the hard constraint overrides the soft constraint
  Point2 goal_pt(1.0, 2.0);
  Point2 start_pt(0.0, 1.0);

  NonlinearFactorGraph graph;
  Symbol x1('x',1);
  graph.emplace_shared<iq2D::PoseXInequality>(x1, 1.0, true);
  graph.emplace_shared<iq2D::PoseYInequality>(x1, 2.0, true);
  graph.emplace_shared<simulated2D::Prior>(start_pt, soft_model2, x1);

  Values initValues;
  initValues.insert(x1, start_pt);

  Values actual = LevenbergMarquardtOptimizer(graph, initValues).optimize();
  Values expected;
  expected.insert(x1, goal_pt);
  CHECK(assert_equal(expected, actual, tol));
}

/* ************************************************************************* */
TEST( testBoundingConstraint, unary_simple_optimization2) {
  // create a single-node graph with a soft and hard constraint to
  // ensure that the hard constraint overrides the soft constraint
  Point2 goal_pt(1.0, 2.0);
  Point2 start_pt(2.0, 3.0);

  NonlinearFactorGraph graph;
  graph.emplace_shared<iq2D::PoseXInequality>(key, 1.0, false);
  graph.emplace_shared<iq2D::PoseYInequality>(key, 2.0, false);
  graph.emplace_shared<simulated2D::Prior>(start_pt, soft_model2, key);

  Values initValues;
  initValues.insert(key, start_pt);

  Values actual = LevenbergMarquardtOptimizer(graph, initValues).optimize();
  Values expected;
  expected.insert(key, goal_pt);
  CHECK(assert_equal(expected, actual, tol));
}

/* ************************************************************************* */
TEST( testBoundingConstraint, MaxDistance_basics) {
  gtsam::Key key1 = 1, key2 = 2;
  Point2 pt1(0,0), pt2(1.0, 0.0), pt3(2.0, 0.0), pt4(3.0, 0.0);
  iq2D::PoseMaxDistConstraint rangeBound(key1, key2, 2.0, mu);
  EXPECT_DOUBLES_EQUAL(2.0, rangeBound.threshold(), tol);
  EXPECT(!rangeBound.isGreaterThan());
  EXPECT(rangeBound.dim() == 1);

  EXPECT(assert_equal((Vector(1) << 2.0).finished(), rangeBound.evaluateError(pt1, pt1)));
  EXPECT(assert_equal(I_1x1, rangeBound.evaluateError(pt1, pt2)));
  EXPECT(assert_equal(Z_1x1, rangeBound.evaluateError(pt1, pt3)));
  EXPECT(assert_equal(-1.0*I_1x1, rangeBound.evaluateError(pt1, pt4)));

  Values config1;
  config1.insert(key1, pt1);
  config1.insert(key2, pt1);
  EXPECT(!rangeBound.active(config1));
  EXPECT(assert_equal(Z_1x1, rangeBound.unwhitenedError(config1)));
  EXPECT(!rangeBound.linearize(config1));
  EXPECT_DOUBLES_EQUAL(0.0, rangeBound.error(config1), tol);

  config1.update(key2, pt2);
  EXPECT(!rangeBound.active(config1));
  EXPECT(assert_equal(Z_1x1, rangeBound.unwhitenedError(config1)));
  EXPECT(!rangeBound.linearize(config1));
  EXPECT_DOUBLES_EQUAL(0.0, rangeBound.error(config1), tol);

  config1.update(key2, pt3);
  EXPECT(rangeBound.active(config1));
  EXPECT(assert_equal(Z_1x1, rangeBound.unwhitenedError(config1)));
  EXPECT_DOUBLES_EQUAL(0.0, rangeBound.error(config1), tol);

  config1.update(key2, pt4);
  EXPECT(rangeBound.active(config1));
  EXPECT(assert_equal(-1.0*I_1x1, rangeBound.unwhitenedError(config1)));
  EXPECT_DOUBLES_EQUAL(0.5*mu, rangeBound.error(config1), tol);
}

/* ************************************************************************* */
TEST( testBoundingConstraint, MaxDistance_simple_optimization) {

  Point2 pt1(0,0), pt2_init(5.0, 0.0), pt2_goal(2.0, 0.0);
  Symbol x1('x',1), x2('x',2);

  NonlinearFactorGraph graph;
  graph.emplace_shared<simulated2D::equality_constraints::UnaryEqualityConstraint>(pt1, x1);
  graph.emplace_shared<simulated2D::Prior>(pt2_init, soft_model2_alt, x2);
  graph.emplace_shared<iq2D::PoseMaxDistConstraint>(x1, x2, 2.0);

  Values initial_state;
  initial_state.insert(x1, pt1);
  initial_state.insert(x2, pt2_init);

  Values expected;
  expected.insert(x1, pt1);
  expected.insert(x2, pt2_goal);

  // FAILS: VectorValues assertion failure
//  Optimizer::shared_values actual = Optimizer::optimizeLM(graph, initial_state);
//  EXPECT(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
TEST( testBoundingConstraint, avoid_demo) {

  Symbol x1('x',1), x2('x',2), x3('x',3), l1('l',1);
  double radius = 1.0;
  Point2 x1_pt(0,0), x2_init(2.0, 0.5), x2_goal(2.0, 1.0), x3_pt(4.0, 0.0), l1_pt(2.0, 0.0);
  Point2 odo(2.0, 0.0);

  NonlinearFactorGraph graph;
  graph.emplace_shared<simulated2D::equality_constraints::UnaryEqualityConstraint>(x1_pt, x1);
  graph.emplace_shared<simulated2D::Odometry>(odo, soft_model2_alt, x1, x2);
  graph.emplace_shared<iq2D::LandmarkAvoid>(x2, l1, radius);
  graph.emplace_shared<simulated2D::equality_constraints::UnaryEqualityPointConstraint>(l1_pt, l1);
  graph.emplace_shared<simulated2D::Odometry>(odo, soft_model2_alt, x2, x3);
  graph.emplace_shared<simulated2D::equality_constraints::UnaryEqualityConstraint>(x3_pt, x3);

  Values init, expected;
  init.insert(x1, x1_pt);
  init.insert(x3, x3_pt);
  init.insert(l1, l1_pt);
  expected = init;
  init.insert(x2, x2_init);
  expected.insert(x2, x2_goal);

  // FAILS: segfaults on optimization
//  Optimizer::shared_values actual = Optimizer::optimizeLM(graph, init);
//  EXPECT(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

