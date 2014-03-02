/**
 * @file    testScalelessBetweenFactor.cpp
 * @brief  
 * @author Luca Carlone
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ScalelessBetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace gtsam::noiseModel;

/* ************************************************************************* */
Vector callEvaluateError(const ScalelessBetweenFactor<Pose3>& factor,const Pose3& P1, const Pose3& P2){
  return factor.evaluateError(P1, P2);
}

TEST(ScalelessBetweenFactor, Error) {
  Pose3 P1 = Pose3();
  Pose3 P2 = Pose3(Rot3(),Point3(1,1,1));

  Unit3 relDirection = Unit3(1,1,1);
  Rot3 relRot = Rot3();
  EssentialMatrix E(relRot,relDirection);
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 1.0);

  ScalelessBetweenFactor<Pose3> factor = ScalelessBetweenFactor<Pose3>(0,1,E,noise);

  Vector ExpectedError(5); ExpectedError << 0,0,0,0,0;
  Vector actualError = factor.evaluateError(P1,P2);
  EXPECT(assert_equal(ExpectedError,actualError));
}

TEST(ScalelessBetweenFactor, Jacobian) {
  Pose3 P1 = Pose3();
  Pose3 P2 = Pose3(Rot3(),Point3(1,1,1));

  Unit3 relDirection = Unit3(1,1,1);
  Rot3 relRot = Rot3();
  EssentialMatrix E(relRot,relDirection);
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 1.0);

  ScalelessBetweenFactor<Pose3> factor = ScalelessBetweenFactor<Pose3>(0,1,E,noise);

  Matrix actualH1, actualH2;
  Vector actual = factor.evaluateError(P1, P2, actualH1, actualH2);

  Matrix numericalH1 = numericalDerivative11<Pose3>(boost::bind(&callEvaluateError, factor, _1, P2), P1);
  EXPECT(assert_equal(numericalH1,actualH1));

  Matrix numericalH2 = numericalDerivative11<Pose3>(boost::bind(&callEvaluateError, factor, P1, _1), P2);
  EXPECT(assert_equal(numericalH2,actualH2));
}

TEST(ScalelessBetweenFactor, Jacobian2) {
  Pose3 P1 = Pose3();
  Pose3 P2 = Pose3(Rot3::rodriguez(0.1, 0.2, 0.3),Point3(1.5,11,1.6));

  Unit3 relDirection = Unit3(1,1,1);
  Rot3 relRot = Rot3();
  EssentialMatrix E(relRot,relDirection);
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 1.0);

  ScalelessBetweenFactor<Pose3> factor = ScalelessBetweenFactor<Pose3>(0,1,E,noise);

  Matrix actualH1, actualH2;
  Vector actual = factor.evaluateError(P1, P2, actualH1, actualH2);

  // suspiciously large tolerances..
  Matrix numericalH1 = numericalDerivative11<Pose3>(boost::bind(&callEvaluateError, factor, _1, P2), P1);
  EXPECT(assert_equal(numericalH1,actualH1,0.1));

  // suspiciously large tolerances..
  Matrix numericalH2 = numericalDerivative11<Pose3>(boost::bind(&callEvaluateError, factor, P1, _1), P2);
  EXPECT(assert_equal(numericalH2,actualH2,1e-4));
}

/* ************************************************************************* *
TEST(ScalelessBetweenFactor, optimization) {
  Pose3 P1 = Pose3();
  Pose3 P2 = Pose3(Rot3(),Point3(0.5,1,0.5));

  NonlinearFactorGraph graph;

  // prior
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Isotropic::Sigma(6, 1.0);
  graph.push_back(PriorFactor<Pose3>(0, P1, priorNoise)); // add directly to graph

  // scaleless between factor
  Unit3 relDirection = Unit3(1,1,1);
  Rot3 relRot = Rot3();
  EssentialMatrix E(relRot,relDirection);
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 1.0);
  ScalelessBetweenFactor<Pose3> factor = ScalelessBetweenFactor<Pose3>(0,1,E,noise);
  graph.push_back(factor);

  Values expected;
  expected.insert(0,P1);
  expected.insert(1,P2);

  Values initial;
  initial.insert(0,P1);
  Pose3 noisePose = Pose3(Rot3::rodriguez(0.5, 0.5, 0.3),Point3(1,-1,1));
  initial.insert(1,P2.compose(noisePose));

  LevenbergMarquardtParams params;
  params.setVerbosityLM("TRYDELTA");
  params.setVerbosity("DELTA");
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  Values actual = lm.optimize();
  EXPECT(assert_equal(expected,actual,0.1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
