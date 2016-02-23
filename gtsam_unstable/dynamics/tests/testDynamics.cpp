/**
 * @file testPoseRTV
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/dynamics/Predictor.h>


using namespace gtsam;

/* x \in X == R^n
 * xdot \in Xdot == R^n
 * u \in U = R^p
 * theta \in Theta == R^m
 */

//n = 3
//p = 2
//m = 5

/* ************************************************************************* */
TEST( testDynamics, example ) {

  NonlinearFactorGraph graph;
//  noiseModel::Diagonal::shared_ptr R = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.3, 0.3, 0.3));
//
//  Expression<Matrix<5>> theta_expr(50);
////  R = dynamics noise model;
//  Matrix<3> X;
////  for (pair<X,U> xu : xu_pairs) {
//  X[0] = 0;
//  X[1] = 0;
//  X[2] = 0;
//
//  Predictor predict(X, 0.1);
//  Expression<X> predict_expr(predict, theta_expr);
//  graph.addExpressionFactor(predict_expr, next_x, R);
}

TEST( testDynamics, testPredict ) {
  Matrix31 X;
  X << 0,0,0;
  Predictor predict(X, 0.1);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

