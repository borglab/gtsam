/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testQPSimple.cpp
 * @brief   Unit tests for testQPSimple
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/LCNLPSolver.h>
#include <gtsam_unstable/nonlinear/NonlinearInequality.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam::symbol_shorthand;
using namespace gtsam;
const double tol = 1e-10;

//******************************************************************************
const size_t X_AXIS = 0;
const size_t Y_AXIS = 1;
const size_t Z_AXIS = 2;

/**
 * Inequality boundary constraint on one axis (x, y or z)
 *      axis <= bound
 */
class AxisUpperBound : public NonlinearInequality1<Pose3> {
  typedef NonlinearInequality1<Pose3> Base;
  size_t axis_;
  double bound_;

public:
  AxisUpperBound(Key key, size_t axis, double bound, Key dualKey) : Base(key, dualKey), axis_(axis), bound_(bound) {
  }

  double computeError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const {
    if (H)
      *H = (Matrix(1,6) << zeros(1,3), pose.rotation().matrix().row(axis_)).finished();
    return pose.translation().vector()[axis_] - bound_;
  }
};

/**
 * Inequality boundary constraint on one axis (x, y or z)
 *      bound <= axis
 */
class AxisLowerBound : public NonlinearInequality1<Pose3> {
  typedef NonlinearInequality1<Pose3> Base;
  size_t axis_;
  double bound_;

public:
  AxisLowerBound(Key key, size_t axis, double bound, Key dualKey) : Base(key, dualKey), axis_(axis), bound_(bound) {
  }

  double computeError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const {
    if (H)
      *H = (Matrix(1,6) << zeros(1,3), -pose.rotation().matrix().row(axis_)).finished();
    return -pose.translation().vector()[axis_] + bound_;
  }
};

//******************************************************************************
TEST(testlcnlpSolver, poseWithABoundary) {
  const Key dualKey = 0;

  //Instantiate LCNLP
  LCNLP lcnlp;
  lcnlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0, 0)), noiseModel::Unit::Create(6)));
  AxisUpperBound constraint(X(1), X_AXIS, 0, dualKey);
  lcnlp.linearInequalities.add(constraint);

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(0.3, 0.2, 0.3), Point3(1, 0, 0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(0, 0, 0)));

  // Instantiate LCNLPSolver
  LCNLPSolver lcnlpSolver(lcnlp);
  Values actualSolution = lcnlpSolver.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
}

//******************************************************************************
TEST(testlcnlpSolver, poseWithNoConstraints) {

  //Instantiate LCNLP
  LCNLP lcnlp;
  lcnlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0, 0)), noiseModel::Unit::Create(6)));

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(0.3, 0.2, 0.3), Point3(1, 0, 0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0, 0)));

  // Instantiate LCNLPSolver
  LCNLPSolver lcnlpSolver(lcnlp);
  Values actualSolution = lcnlpSolver.optimize(initialValues, true, false).first; // TODO: Fails without warmstart

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
}

//******************************************************************************
TEST(testlcnlpSolver, poseWithinA2DBox) {
  const Key dualKey = 0;

  //Instantiate LCNLP
  LCNLP lcnlp;
  lcnlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(10, 0.5, 0)), noiseModel::Unit::Create(6)));
  lcnlp.linearInequalities.add(AxisLowerBound(X(1), X_AXIS, -1, dualKey)); // -1 <= x
  lcnlp.linearInequalities.add(AxisUpperBound(X(1), X_AXIS, 1, dualKey+1)); //      x <= 1
  lcnlp.linearInequalities.add(AxisLowerBound(X(1), Y_AXIS, -1, dualKey+2)); // -1 <= y
  lcnlp.linearInequalities.add(AxisUpperBound(X(1), Y_AXIS, 1, dualKey+3));//         y <= 1

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(1, -1, 2), Point3(3, -5, 0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0.5, 0)));

  // Instantiate LCNLPSolver
  LCNLPSolver lcnlpSolver(lcnlp);
  Values actualSolution = lcnlpSolver.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
}

//******************************************************************************
TEST(testlcnlpSolver, posesInA2DBox) {
  const double xLowerBound = -3.0,
      xUpperBound = 5.0,
      yLowerBound = -1.0,
      yUpperBound = 2.0,
      zLowerBound = 0.0,
      zUpperBound = 2.0;

  //Instantiate LCNLP
  LCNLP lcnlp;

  // prior on the first pose
  SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001).finished());
  lcnlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(), priorNoise));

  // odometry between factor for subsequent poses
  SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.001, 0.001, 0.001, 0.1, 0.1, 0.1).finished());
  Pose3 odo12(Rot3::ypr(M_PI/2.0, 0, 0), Point3(10, 0, 0));
  lcnlp.cost.add(BetweenFactor<Pose3>(X(1), X(2), odo12, odoNoise));

  Pose3 odo23(Rot3::ypr(M_PI/2.0, 0, 0), Point3(2, 0, 2));
  lcnlp.cost.add(BetweenFactor<Pose3>(X(2), X(3), odo23, odoNoise));

  // Box constraints
  Key dualKey = 0;
  for (size_t i=1; i<=3; ++i) {
    lcnlp.linearInequalities.add(AxisLowerBound(X(i), X_AXIS, xLowerBound, dualKey++));
    lcnlp.linearInequalities.add(AxisUpperBound(X(i), X_AXIS, xUpperBound, dualKey++));
    lcnlp.linearInequalities.add(AxisLowerBound(X(i), Y_AXIS, yLowerBound, dualKey++));
    lcnlp.linearInequalities.add(AxisUpperBound(X(i), Y_AXIS, yUpperBound, dualKey++));
    lcnlp.linearInequalities.add(AxisLowerBound(X(i), Z_AXIS, zLowerBound, dualKey++));
    lcnlp.linearInequalities.add(AxisUpperBound(X(i), Z_AXIS, zUpperBound, dualKey++));
  }

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3(), Point3(0, 0, 0)));
  initialValues.insert(X(2), Pose3(Rot3(), Point3(0, 0, 0)));
  initialValues.insert(X(3), Pose3(Rot3(), Point3(0, 0, 0)));


  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3());
  expectedSolution.insert(X(2), Pose3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(5, 0, 0)));
  expectedSolution.insert(X(3), Pose3(Rot3::ypr(M_PI, 0, 0), Point3(5, 2, 2)));

  // Instantiate LCNLPSolver
  LCNLPSolver lcnlpSolver(lcnlp);
  bool useWarmStart = true;
  Values actualSolution = lcnlpSolver.optimize(initialValues, useWarmStart).first;

//  cout << "Rotation angles: " << endl;
//  for (size_t i = 1; i<=3; i++) {
//    cout << actualSolution.at<Pose3>(X(i)).rotation().ypr().transpose()*180/M_PI << endl;
//  }

//  cout << "Actual Error: " << lcnlp.cost.error(actualSolution) << endl;
//  cout << "Expected Error: " << lcnlp.cost.error(expectedSolution) << endl;
//  actualSolution.print("actualSolution: ");

  AxisLowerBound factor(X(1), X_AXIS, xLowerBound, dualKey++);
  Matrix hessian = numericalHessian<Pose3>(boost::bind(&AxisLowerBound::computeError, factor, _1, boost::none), Pose3(), 1e-3);
  cout << "Hessian: \n" << hessian << endl;
  CHECK(assert_equal(expectedSolution, actualSolution, 1e-5));
}

//******************************************************************************
TEST(testlcnlpSolver, iterativePoseinBox) {
  const double xLowerBound = -1.0,
      xUpperBound = 1.0,
      yLowerBound = -1.0,
      yUpperBound = 1.0,
      zLowerBound = -1.0,
      zUpperBound = 1.0;

  //Instantiate LCNLP
  LCNLP lcnlp;

  // prior on the first pose
  SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001).finished());
  lcnlp.cost.add(PriorFactor<Pose3>(X(0), Pose3(), priorNoise));

  // odometry between factor for subsequent poses
  SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.001, 0.001, 0.001, 0.1, 0.1, 0.1).finished());
  Pose3 odo(Rot3::ypr(0, 0, 0), Point3(0.4, 0, 0));

  Values initialValues;
  Values isamValues;
  initialValues.insert(X(0), Pose3(Rot3(), Point3(0, 0, 0)));
  isamValues.insert(X(0), Pose3(Rot3(), Point3(0, 0, 0)));
  std::pair<Values, VectorValues> solutionAndDuals;
  solutionAndDuals.first.insert(X(0), Pose3(Rot3(), Point3(0, 0, 0)));
  Values actualSolution;
  bool useWarmStart = true;
  bool firstTime = true;

  // Box constraints
  Key dualKey = 0;
  for (size_t i=1; i<=4; ++i) {

    lcnlp.cost.add(BetweenFactor<Pose3>(X(i-1), X(i), odo, odoNoise));
    lcnlp.linearInequalities.add(AxisLowerBound(X(i), X_AXIS, xLowerBound, dualKey++));
    lcnlp.linearInequalities.add(AxisUpperBound(X(i), X_AXIS, xUpperBound, dualKey++));
    lcnlp.linearInequalities.add(AxisLowerBound(X(i), Y_AXIS, yLowerBound, dualKey++));
    lcnlp.linearInequalities.add(AxisUpperBound(X(i), Y_AXIS, yUpperBound, dualKey++));
    lcnlp.linearInequalities.add(AxisLowerBound(X(i), Z_AXIS, zLowerBound, dualKey++));
    lcnlp.linearInequalities.add(AxisUpperBound(X(i), Z_AXIS, zUpperBound, dualKey++));

    initialValues.insert(X(i), Pose3(Rot3(), Point3(0, 0, 0)));
    isamValues = solutionAndDuals.first;
    isamValues.insert(X(i), solutionAndDuals.first.at(X(i-1)));
    isamValues.print("iSAM values: \n");


    // Instantiate LCNLPSolver
    LCNLPSolver lcnlpSolver(lcnlp);
//    if (firstTime) {
      solutionAndDuals = lcnlpSolver.optimize(isamValues, useWarmStart);
//      firstTime = false;
//    }
//    else {
//      cout << " using this \n ";
//      solutionAndDuals = lcnlpSolver.optimize(isamValues, solutionAndDuals.second, useWarmStart);
//
//    }
    cout << " ************************** \n";
  }
  actualSolution = solutionAndDuals.first;
  cout << "out of loop" << endl;
  Values expectedSolution;
  expectedSolution.insert(X(0), Pose3());
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0, 0, 0), Point3(0.25, 0, 0)));
  expectedSolution.insert(X(2), Pose3(Rot3::ypr(0, 0, 0), Point3(0.50, 0, 0)));
  expectedSolution.insert(X(3), Pose3(Rot3::ypr(0, 0, 0), Point3(0.75, 0, 0)));
  expectedSolution.insert(X(4), Pose3(Rot3::ypr(0, 0, 0), Point3(1, 0, 0)));



//  cout << "Rotation angles: " << endl;
//  for (size_t i = 1; i<=3; i++) {
//    cout << actualSolution.at<Pose3>(X(i)).rotation().ypr().transpose()*180/M_PI << endl;
//  }

  cout << "Actual Error: " << lcnlp.cost.error(actualSolution) << endl;
  cout << "Expected Error: " << lcnlp.cost.error(expectedSolution) << endl;
  actualSolution.print("actualSolution: ");

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-5));
}


//******************************************************************************
double testHessian(const Pose3& X) {
  return X.transform_from(Point3(0,0,0)).x();
}

Pose3 pose(Rot3::ypr(0.1, 0.4, 0.7), Point3(1, 9.5, 7.3));
TEST(testlcnlpSolver, testingHessian) {
  Matrix H = numericalHessian(testHessian, pose, 1e-5);
}

double h3(const Vector6& v) {
  return pose.retract(v).translation().x();
}

TEST(testlcnlpSolver, NumericalHessian3) {
  Matrix6 expected;
  expected.setZero();
  Vector6 z;
  z.setZero();
  EXPECT(assert_equal(expected, numericalHessian(h3, z, 1e-5), 1e-5));
}

//******************************************************************************
int main() {
  cout<<"here: "<<endl;
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
