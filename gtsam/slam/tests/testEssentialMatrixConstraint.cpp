/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testEssentialMatrixConstraint.cpp
 *  @brief Unit tests for EssentialMatrixConstraint Class
 *  @author Frank Dellaert
 *  @author Pablo Alcantarilla
 *  @date Jan 5, 2014
 */

#include <gtsam/slam/EssentialMatrixConstraint.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( EssentialMatrixConstraint, test ) {
  // Create a factor
  Key poseKey1(1);
  Key poseKey2(2);
  Rot3 trueRotation = Rot3::RzRyRx(0.15, 0.15, -0.20);
  Point3 trueTranslation(+0.5, -1.0, +1.0);
  Unit3 trueDirection(trueTranslation);
  EssentialMatrix measurement(trueRotation, trueDirection);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 0.25);
  EssentialMatrixConstraint factor(poseKey1, poseKey2, measurement, model);

  // Create a linearization point at the zero-error point
  Pose3 pose1(Rot3::RzRyRx(0.00, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));
  Pose3 pose2(
      Rot3::RzRyRx(0.179693265735950, 0.002945368776519, 0.102274823253840),
      Point3(-3.37493895, 6.14660244, -8.93650986));

  Vector expected = zero(5);
  Vector actual = factor.evaluateError(pose1,pose2);
  CHECK(assert_equal(expected, actual, 1e-8));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, _1, pose2,
          boost::none, boost::none), pose1);
  Matrix expectedH2 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, pose1, _1,
          boost::none, boost::none), pose2);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  Matrix actualH2;
  factor.evaluateError(pose1, pose2, actualH1, actualH2);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-9));
  CHECK(assert_equal(expectedH2, actualH2, 1e-9));
}

EssentialMatrix EssentialFrom2Poses(const Pose3& p1, const Pose3& p2)
{
  Pose3 _1P2_ = p1.between(p2);
  return EssentialMatrix::FromPose3(_1P2_);
}

/* ************************************************************************* */
TEST(EssentialMatrixConstraint, optimization) {
  Pose3 P1true = Pose3();
  Pose3 P2true = Pose3(Rot3(),Point3(1,1,1));

  Pose3 noisePose = Pose3(Rot3::rodriguez(0.5, 0.5, 0.3),Point3(1,-1,1));
  //Pose3 noisePose = Pose3(Rot3::rodriguez(0.0, 0.0, 0.0),Point3(0.01,-0.01,0.01));
  Pose3 P1(P1true);
  Pose3 P2 = P2true.compose(noisePose);

  // test between
  Pose3 expectedBetween = P1.inverse() * P2;
  Matrix actualDBetween1,actualDBetween2;
  Pose3 actualBetween = P1.between(P2, actualDBetween1,actualDBetween2);
  EXPECT(assert_equal(expectedBetween,actualBetween));

  Matrix numericalH1 = numericalDerivative21(testing::between<Pose3> , P1, P2);
  EXPECT(assert_equal(numericalH1,actualDBetween1,5e-3));
  Matrix numericalH2 = numericalDerivative22(testing::between<Pose3> , P1, P2);
  EXPECT(assert_equal(numericalH2,actualDBetween2,1e-5));

  // test fromPose3
  Matrix actualH;
  EssentialMatrix hx = EssentialMatrix::FromPose3(expectedBetween, actualH);
  Matrix expectedH = numericalDerivative11<EssentialMatrix, Pose3>(
      boost::bind(EssentialMatrix::FromPose3, _1, boost::none), expectedBetween);
  EXPECT(assert_equal(expectedH, actualH, 1e-7));

  // check chain rule:
  Matrix expectedHp1 = numericalDerivative11<EssentialMatrix, Pose3>(
        boost::bind(&EssentialFrom2Poses, _1, P2), P1);
  Matrix expectedHp2 = numericalDerivative11<EssentialMatrix, Pose3>(
        boost::bind(&EssentialFrom2Poses, P1, _1), P2);
  EXPECT(assert_equal(expectedHp1, actualH * actualDBetween1, 1e-7));
  EXPECT(assert_equal(expectedHp2, actualH * actualDBetween2, 1e-7));

  // EssentialMatrixConstraint
  Pose3 betweenTrue = P1true.between(P2true);
  Unit3 expectedDirection = Unit3(betweenTrue.translation());
  Rot3 expectedRotation = betweenTrue.rotation();

  EssentialMatrix E(expectedRotation,expectedDirection);
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 1.0);
  EssentialMatrixConstraint factor(1, 2, E, noise);

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, _1, P2,
          boost::none, boost::none), P1);
  Matrix expectedH2 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, P1, _1,
          boost::none, boost::none), P2);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2;
  factor.evaluateError(P1, P2, actualH1, actualH2);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH1, actualH1, 1e-9));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-9));

  // OPTIMIZATION
  NonlinearFactorGraph graph;
  graph.push_back(factor);

  // prior
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Isotropic::Sigma(6, 1.0);
  graph.push_back(PriorFactor<Pose3>(1, P1, priorNoise));

  Values initial;
  initial.insert(1,P1);
  initial.insert(2,P2);

  LevenbergMarquardtParams params;
  //params.relativeErrorTol = 0.0;
  //params.absoluteErrorTol = 0.0;
  //params.setVerbosityLM("TRYDELTA");
  //params.setVerbosity("DELTA");
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  Values actual = lm.optimize();

  Rot3 actualRotation = actual.at<Pose3>(2).rotation();
  Unit3 actualDirection(actual.at<Pose3>(2).translation());

  EXPECT(assert_equal(P1true,actual.at<Pose3>(1),1e-4));
  EXPECT(assert_equal(expectedRotation,actualRotation,1e-4));
  EXPECT(assert_equal(expectedDirection,actualDirection,1e-4));
}

#include <gtsam/inference/Symbol.h>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
#include <gtsam/navigation/AttitudeFactor.h>
#include <fstream>
#include <iostream>

Values loadInitialGuess(const string& filename) {
  Values initialValues;

  // Load the data file
  ifstream fin(filename.c_str(),ifstream::in);
  if(!fin)
    cout << "Error in loadInitialGuess: can not find the file!!" << endl;

  int pose_id;
  while (fin >> pose_id) {
    Matrix3 rotationMatrix;
    fin >> rotationMatrix(0,0);
    fin >> rotationMatrix(0,1);
    fin >> rotationMatrix(0,2);
    fin >> rotationMatrix(1,0);
    fin >> rotationMatrix(1,1);
    fin >> rotationMatrix(1,2);
    fin >> rotationMatrix(2,0);
    fin >> rotationMatrix(2,1);
    fin >> rotationMatrix(2,2);
    Rot3 R = Rot3(rotationMatrix);

    Vector3 translationVector;
    for (size_t i = 0; i < 3; ++i) { // fill in translation
      fin >> translationVector(i);
    }
    Point3 t = Point3(translationVector);
    initialValues.insert(pose_id, Pose3(R,t));
  }
  fin.close();
  return initialValues;
}


bool loadBetween(const string& filename, NonlinearFactorGraph& graph) {

  // Load the data file
  ifstream fin(filename.c_str(),ifstream::in);
  if(!fin){
    cout << "Error in loadBetween: can not find the file!!" << endl;
    return false;
  }

  // noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 0.1);

  const noiseModel::Robust::shared_ptr noise =
      noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(2.0),noiseModel::Isotropic::Sigma(5, 0.05));

  int id1, id2;
  while (fin >> id1) {
    fin >> id2;

    Matrix3 rotationMatrix;
    fin >> rotationMatrix(0,0);
    fin >> rotationMatrix(0,1);
    fin >> rotationMatrix(0,2);
    fin >> rotationMatrix(1,0);
    fin >> rotationMatrix(1,1);
    fin >> rotationMatrix(1,2);
    fin >> rotationMatrix(2,0);
    fin >> rotationMatrix(2,1);
    fin >> rotationMatrix(2,2);
    Rot3 R_E = Rot3(rotationMatrix);

    Vector3 directionVector;
    for (size_t i = 0; i < 3; ++i) { // fill in translation
      fin >> directionVector(i);
    }
    Unit3 t_E = Unit3(Point3(directionVector));
    EssentialMatrix E(R_E,t_E);
    EssentialMatrixConstraint factor(id1, id2, E, noise);
    graph.push_back(factor);

  }
  fin.close();
  return true;
}

bool loadGravity(const string& filename, NonlinearFactorGraph& graph) {

  // Load the data file
  ifstream fin(filename.c_str(),ifstream::in);
  if(!fin){
    cout << "Error in loadGravity: can not find the file!!" << endl;
    return false;
  }

  Unit3 g_global(Point3(0,0,-1));\
  g_global.print("g_global ");

  int id1;
  while (fin >> id1) {
    Vector3 gravityVector;
    for (size_t i = 0; i < 3; ++i) {
      fin >> gravityVector(i);
    }
    Unit3 g_local = Unit3(Point3(gravityVector));
    SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.1);
    Pose3AttitudeFactor factor(id1, g_local, model, g_global);
    graph.push_back(factor);
  }
  fin.close();
  return true;
}

bool loadPriors(const string& filename, NonlinearFactorGraph& graph) {

  // Load the data file
  ifstream fin(filename.c_str(),ifstream::in);
  if(!fin){
    cout << "Error in loadPriors: can not find the file!!" << endl;
    return false;
  }

  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Precisions((Vector(6) << 1e5,1e5,1e5,0,0,0));

  int pose_id;
  while (fin >> pose_id) {
    Matrix3 rotationMatrix;
    fin >> rotationMatrix(0,0);
    fin >> rotationMatrix(0,1);
    fin >> rotationMatrix(0,2);
    fin >> rotationMatrix(1,0);
    fin >> rotationMatrix(1,1);
    fin >> rotationMatrix(1,2);
    fin >> rotationMatrix(2,0);
    fin >> rotationMatrix(2,1);
    fin >> rotationMatrix(2,2);
    Rot3 R = Rot3(rotationMatrix);

    Vector3 translationVector;
    for (size_t i = 0; i < 3; ++i) { // fill in translation
      fin >> translationVector(i);
    }
    Point3 t = Point3(translationVector);
    Pose3 pose_matlab_i = Pose3(R,t);
    graph.push_back(PriorFactor<Pose3>(pose_id, pose_matlab_i, priorNoise));
  }
  fin.close();
  return true;
}


bool writeEstimate(const string& filename, const Values& estimate) {

  // Open the output file
  ofstream os;
  os.open(filename.c_str());
  os.precision(20);
  if (!os.is_open()) {
    cout << "Error in writeEstimate: can not open the file!!" << endl;
    return false;
  }

  for(size_t i=0; i<estimate.size(); ++i){ // for all poses
    Pose3 estimate_i = estimate.at<Pose3>(i);
    Rot3 R_i   = estimate_i.rotation();
    Point3 t_i = estimate_i.translation();
    Matrix3 rotationMatrix = R_i.matrix();
    os << rotationMatrix(0,0) << " ";
    os << rotationMatrix(0,1) << " ";
    os << rotationMatrix(0,2) << " ";
    os << rotationMatrix(1,0) << " ";
    os << rotationMatrix(1,1) << " ";
    os << rotationMatrix(1,2) << " ";
    os << rotationMatrix(2,0) << " ";
    os << rotationMatrix(2,1) << " ";
    os << rotationMatrix(2,2) << " ";

    Vector3 translationVector = t_i.vector();
    os << translationVector(0) << " ";
    os << translationVector(1) << " ";
    os << translationVector(2);
    os << endl;
  }

  os.close();
  return true;
}

/* ************************************************************************* */
TEST(EssentialMatrixConstraint, BAinSO2) {

  // create initial values
  string initialGuessFile = "/home/aspn/borg/BAinSO2/data/initialGuessNL.txt";
  Values initialValues = loadInitialGuess(initialGuessFile);
  // initialValues.print("initialValues\n");

  // create EssentialMatrixConstraints
  string betweenFile = "/home/aspn/borg/BAinSO2/data/betweenNL.txt";
  NonlinearFactorGraph graph;
  loadBetween(betweenFile, graph);
  // graph.print("");

  // create EssentialMatrixConstraints
  string gravityFile = "/home/aspn/borg/BAinSO2/data/gravityNL.txt";
  loadGravity(gravityFile, graph);
  //graph.print("");

  // add priors on rotations
  loadPriors(initialGuessFile, graph);
  //graph.print("");

  LevenbergMarquardtParams params;
  params.relativeErrorTol = 0.0;
  params.absoluteErrorTol = 0.0;
  params.setVerbosityLM("TRYLAMBDA");
  //params.setVerbosity("DELTA");
  LevenbergMarquardtOptimizer lm(graph, initialValues, params);
  Values estimate = lm.optimize();

  // write results
  string outputFile = "/home/aspn/borg/BAinSO2/data/estimateNL.txt";
  bool success = writeEstimate(outputFile, estimate);
  std::cout << "success: " << success << std::endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

