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
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
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
  Vector5 sigmas;
  sigmas << 0.3,0.3,0.3,0.3,0.3;
  noiseModel::Diagonal::shared_ptr R = noiseModel::Diagonal::Sigmas(sigmas);

  Values vals;
  Matrix31 Theta;
  Theta << 0.1,0.1,0.1;

  vals.insert(50, Theta);
  Expression<Matrix51> theta_expr(50);
  Matrix31 X, X2;
  X << 0,0,0;
  X2 << 0.2,0.2,0.2;

  Predictor predict(X, 0.1);
  Expression<Matrix31> predict_expr(predict, theta_expr);
  graph.addExpressionFactor(R, X2, predict_expr);
  graph.print("Factor Graph:\n");
  LevenbergMarquardtOptimizer optimizer(graph, vals);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");
}

TEST( testDynamics, testPredict ) {
  Matrix31 X;
  X << 0,0,0;
  Predictor predict(X, 0.1);
  Matrix51 Theta;
  Theta << 0.1,0.1,0.1,0.1,0.1;
  Matrix31 X2;
  Matrix35 J;
  X2 = predict(Theta, J);
  std::cout << "X1\n" << X << std::endl;
  std::cout << "X2\n" << X2 << std::endl;

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

