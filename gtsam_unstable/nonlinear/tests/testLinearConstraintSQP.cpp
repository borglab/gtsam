/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testLinearConstraintSQP.cpp
 * @brief   Unit tests for testLinearConstraintSQP
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
#include <gtsam_unstable/nonlinear/LinearConstraintSQP.h>
#include <gtsam_unstable/nonlinear/LinearInequalityFactor.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam::symbol_shorthand;
using namespace gtsam;
const double tol = 1e-10;

//******************************************************************************
// x + y - 1 = 0
class ConstraintProblem1 : public LinearEqualityFactor2<double, double> {
  typedef LinearEqualityFactor2<double, double> Base;

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

  EqualityFactorGraph equalities;
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

  //Instantiate LinearConstraintNLP
  LinearConstraintNLP lcnlp;
  Values linPoint;
  linPoint.insert<Vector1>(X(1), zero(1));
  linPoint.insert<Vector1>(Y(1), zero(1));
  lcnlp.cost.add(LinearContainerFactor(hf, linPoint)); // wrap it using linearcontainerfactor
  lcnlp.linearEqualities.add(ConstraintProblem1(X(1), Y(1), dualKey));

  Values initialValues;
  initialValues.insert(X(1), 0.0);
  initialValues.insert(Y(1), 0.0);

  // Instantiate LinearConstraintSQP
  LinearConstraintSQP lcnlpSolver(lcnlp);
  Values actualValues = lcnlpSolver.optimize(initialValues).first;

  DOUBLES_EQUAL(expectedSolution.at(X(1))[0], actualValues.at<double>(X(1)), 1e-100);
  DOUBLES_EQUAL(expectedSolution.at(Y(1))[0], actualValues.at<double>(Y(1)), 1e-100);

}

//******************************************************************************
/**
 * A simple linear constraint on Pose3's x coordinate enforcing x==0
 */
class LineConstraintX : public LinearEqualityFactor1<Pose3> {
  typedef LinearEqualityFactor1<Pose3> Base;
public:
  LineConstraintX(Key key, Key dualKey) : Base(key, dualKey, 1) {
  }

  Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const {
    if (H)
      *H = (Matrix(1,6) << zeros(1,3), pose.rotation().matrix().row(0)).finished();
    return (Vector(1) << pose.x()).finished();
  }
};

TEST(testlcnlpSolver, poseOnALine) {
  const Key dualKey = 0;


  //Instantiate LinearConstraintNLP
  LinearConstraintNLP lcnlp;
  lcnlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::Ypr(0.1, 0.2, 0.3), Point3(1, 0, 0)), noiseModel::Unit::Create(6)));
  LineConstraintX constraint(X(1), dualKey);
  lcnlp.linearEqualities.add(constraint);

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::Ypr(0.3, 0.2, 0.3), Point3(1,0,0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::Ypr(0.1, 0.2, 0.3), Point3()));

  // Instantiate LinearConstraintSQP
  LinearConstraintSQP lcnlpSolver(lcnlp);
  Values actualSolution = lcnlpSolver.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
}

//******************************************************************************
/// x + y - 1 <= 0
class InequalityProblem1 : public LinearInequalityFactor2<double, double> {
  typedef LinearInequalityFactor2<double, double> Base;
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

  InequalityFactorGraph inequalities;
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

  //Instantiate LinearConstraintNLP
  LinearConstraintNLP lcnlp;
  Values linPoint;
  linPoint.insert<Vector1>(X(1), zero(1));
  linPoint.insert<Vector1>(Y(1), zero(1));
  lcnlp.cost.add(LinearContainerFactor(hf, linPoint)); // wrap it using linearcontainerfactor
  lcnlp.linearInequalities.add(InequalityProblem1(X(1), Y(1), dualKey));

  Values initialValues;
  initialValues.insert(X(1), 1.0);
  initialValues.insert(Y(1), -10.0);

  // Instantiate LinearConstraintSQP
  LinearConstraintSQP lcnlpSolver(lcnlp);
  Values actualValues = lcnlpSolver.optimize(initialValues).first;

  DOUBLES_EQUAL(expectedSolution.at(X(1))[0], actualValues.at<double>(X(1)), 1e-10);
  DOUBLES_EQUAL(expectedSolution.at(Y(1))[0], actualValues.at<double>(Y(1)), 1e-10);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
