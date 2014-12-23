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
// x + y - 1 = 0
class ConstraintProblem1 : public NonlinearConstraint2<double, double> {
  typedef NonlinearConstraint2<double, double> Base;

public:
  ConstraintProblem1(Key xK, Key yK, Key dualKey) : Base(xK, yK, dualKey, 1) {}

  // x + y - 1
  Vector evaluateError(const double& x, const double& y,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    if (H1) *H1 = eye(1);
    if (H2) *H2 = eye(1);
    return (Vector(1) << x + y - 1.0).finished();
  }
};

TEST(testlcnlpSolver, QPProblem) {
  const Key dualKey = 0;

  // Simple quadratic cost: x1^2 + x2^2
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence here we have G11 = 2, G12 = 0, G22 = 2, g1 = 0, g2 = 0, f = 0
  HessianFactor hf(X(1), Y(1), 2.0 * ones(1,1), zero(1), zero(1),
                    2*ones(1,1), zero(1) , 0);

  LinearEqualityFactorGraph equalities;
  LinearEquality linearConstraint(X(1), ones(1), Y(1), ones(1), 1*ones(1), dualKey); // x + y - 1 = 0
  equalities.push_back(linearConstraint);

  // Compare against QP
  QP qp;
  qp.cost.add(hf);
  qp.equalities = equalities;

  // instantiate QPsolver
  QPSolver qpSolver(qp);
  // create initial values for optimization
  VectorValues initialVectorValues;
  initialVectorValues.insert(X(1), zero(1));
  initialVectorValues.insert(Y(1), ones(1));
  VectorValues expectedSolution = qpSolver.optimize(initialVectorValues).first;

  //Instantiate LCNLP
  LCNLP lcnlp;
  Values linPoint;
  linPoint.insert<Vector1>(X(1), zero(1));
  linPoint.insert<Vector1>(Y(1), zero(1));
  lcnlp.cost.add(LinearContainerFactor(hf, linPoint)); // wrap it using linearcontainerfactor
  lcnlp.linearEqualities.add(ConstraintProblem1(X(1), Y(1), dualKey));

  Values initialValues;
  initialValues.insert(X(1), 0.0);
  initialValues.insert(Y(1), 0.0);

  // Instantiate LCNLPSolver
  LCNLPSolver lcnlpSolver(lcnlp);
  Values actualValues = lcnlpSolver.optimize(initialValues).first;

  DOUBLES_EQUAL(expectedSolution.at(X(1))[0], actualValues.at<double>(X(1)), 1e-100);
  DOUBLES_EQUAL(expectedSolution.at(Y(1))[0], actualValues.at<double>(Y(1)), 1e-100);

}

//******************************************************************************
class LineConstraintX : public NonlinearConstraint1<Pose3> {
  typedef NonlinearConstraint1<Pose3> Base;
public:
  LineConstraintX(Key key, Key dualKey) : Base(key, dualKey, 1) {
  }

  double computeError(const Pose3& pose) const {
    return pose.x();
  }

  Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const {
    if (H)
      *H = (Matrix(1,6) << zeros(1,3), pose.rotation().matrix().row(0)).finished();
    return (Vector(1) << pose.x()).finished();
  }
};

TEST(testlcnlpSolver, poseOnALine) {
  const Key dualKey = 0;


  //Instantiate LCNLP
  LCNLP lcnlp;
  lcnlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0, 0)), noiseModel::Unit::Create(6)));
  LineConstraintX constraint(X(1), dualKey);
  lcnlp.linearEqualities.add(constraint);

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(0.3, 0.2, 0.3), Point3(1,0,0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3()));

  // Instantiate LCNLPSolver
  LCNLPSolver lcnlpSolver(lcnlp);
  Values actualSolution = lcnlpSolver.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
  Pose3 pose(Rot3::ypr(0.1, 0.2, 0.3), Point3());
  Matrix hessian = numericalHessian<Pose3>(boost::bind(&LineConstraintX::computeError, constraint, _1), pose, 1e-2);
}

//******************************************************************************
/// x + y - 1 <= 0
class InequalityProblem1 : public NonlinearInequality2<double, double> {
  typedef NonlinearInequality2<double, double> Base;
public:
  InequalityProblem1(Key xK, Key yK, Key dualKey) : Base(xK, yK, dualKey) {}

  double computeError(const double& x, const double& y,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    if (H1) *H1 = eye(1);
    if (H2) *H2 = eye(1);
    return x + y - 1.0;
  }
};

TEST(testlcnlpSolver, inequalityConstraint) {
  const Key dualKey = 0;

  // Simple quadratic cost: x^2 + y^2
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence here we have G11 = 2, G12 = 0, G22 = 2, g1 = 0, g2 = 0, f = 0
  HessianFactor hf(X(1), Y(1), 2.0 * ones(1,1), zero(1), zero(1),
                    2*ones(1,1), zero(1) , 0);

  LinearInequalityFactorGraph inequalities;
  LinearInequality linearConstraint(X(1), ones(1), Y(1), ones(1), 1.0, dualKey); // x + y - 1 <= 0
  inequalities.push_back(linearConstraint);

  // Compare against QP
  QP qp;
  qp.cost.add(hf);
  qp.inequalities = inequalities;

  // instantiate QPsolver
  QPSolver qpSolver(qp);
  // create initial values for optimization
  VectorValues initialVectorValues;
  initialVectorValues.insert(X(1), zero(1));
  initialVectorValues.insert(Y(1), zero(1));
  VectorValues expectedSolution = qpSolver.optimize(initialVectorValues).first;

  //Instantiate LCNLP
  LCNLP lcnlp;
  Values linPoint;
  linPoint.insert<Vector1>(X(1), zero(1));
  linPoint.insert<Vector1>(Y(1), zero(1));
  lcnlp.cost.add(LinearContainerFactor(hf, linPoint)); // wrap it using linearcontainerfactor
  lcnlp.linearInequalities.add(InequalityProblem1(X(1), Y(1), dualKey));

  Values initialValues;
  initialValues.insert(X(1), 1.0);
  initialValues.insert(Y(1), -10.0);

  // Instantiate LCNLPSolver
  LCNLPSolver lcnlpSolver(lcnlp);
  Values actualValues = lcnlpSolver.optimize(initialValues).first;

  DOUBLES_EQUAL(expectedSolution.at(X(1))[0], actualValues.at<double>(X(1)), 1e-10);
  DOUBLES_EQUAL(expectedSolution.at(Y(1))[0], actualValues.at<double>(Y(1)), 1e-10);
}

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
  Values actualSolution = lcnlpSolver.optimize(initialValues).first;

//  cout << "Rotation angles: " << endl;
//  for (size_t i = 1; i<=3; i++) {
//    cout << actualSolution.at<Pose3>(X(i)).rotation().ypr().transpose()*180/M_PI << endl;
//  }

//  cout << "Actual Error: " << lcnlp.cost.error(actualSolution) << endl;
//  cout << "Expected Error: " << lcnlp.cost.error(expectedSolution) << endl;
//  actualSolution.print("actualSolution: ");

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-5));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
