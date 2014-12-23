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
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/SQPSimple.h>
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

TEST(testSQPSimple, QPProblem) {
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

  //Instantiate NLP
  NLP nlp;
  Values linPoint;
  linPoint.insert<Vector1>(X(1), zero(1));
  linPoint.insert<Vector1>(Y(1), zero(1));
  nlp.cost.add(LinearContainerFactor(hf, linPoint)); // wrap it using linearcontainerfactor
  nlp.linearEqualities.add(ConstraintProblem1(X(1), Y(1), dualKey));

  Values initialValues;
  initialValues.insert(X(1), 0.0);
  initialValues.insert(Y(1), 0.0);

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
  Values actualValues = sqpSimple.optimize(initialValues).first;

  DOUBLES_EQUAL(expectedSolution.at(X(1))[0], actualValues.at<double>(X(1)), 1e-100);
  DOUBLES_EQUAL(expectedSolution.at(Y(1))[0], actualValues.at<double>(Y(1)), 1e-100);

}

//******************************************************************************
class CircleConstraint : public NonlinearConstraint2<double, double> {
  typedef NonlinearConstraint2<double, double> Base;
public:
  CircleConstraint(Key xK, Key yK, Key dualKey) : Base(xK, yK, dualKey, 1) {}

  Vector evaluateError(const double& x, const double& y,
    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
        boost::none) const {
    if (H1) *H1 = (Matrix(1,1) << 2*(x-1)).finished();
    if (H2) *H2 = (Matrix(1,1) << 2*y).finished();
    return (Vector(1) << (x-1)*(x-1) + y*y - 0.25).finished();
  }

  void evaluateHessians(const double& x, const double& y,
        std::vector<Matrix>& G11, std::vector<Matrix>& G12,
        std::vector<Matrix>& G22) const {
    G11.push_back((Matrix(1,1) << 2).finished());
    G12.push_back((Matrix(1,1) << 0).finished());
    G22.push_back((Matrix(1,1) << 2).finished());
  }

};

TEST_UNSAFE(testSQPSimple, quadraticCostNonlinearConstraint) {
  const Key dualKey = 0;

  //Instantiate NLP
  NLP nlp;

  // Simple quadratic cost: x1^2 + x2^2 +1000
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence here we have G11 = 2, G12 = 0, G22 = 2, g1 = 0, g2 = 0, f = 0
  Values linPoint;
  linPoint.insert<Vector1>(X(1), zero(1));
  linPoint.insert<Vector1>(Y(1), zero(1));
  HessianFactor hf(X(1), Y(1), 2.0 * ones(1,1), zero(1), zero(1),
                    2*ones(1,1), zero(1) , 1000);
  nlp.cost.add(LinearContainerFactor(hf, linPoint)); // wrap it using linearcontainerfactor
  nlp.nonlinearEqualities.add(CircleConstraint(X(1), Y(1), dualKey));

  Values initialValues;
  initialValues.insert<double>(X(1), 4.0);
  initialValues.insert<double>(Y(1), 10.0);

  Values expectedSolution;
  expectedSolution.insert<double>(X(1), 0.5);
  expectedSolution.insert<double>(Y(1), 0.0);

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
  Values actualSolution = sqpSimple.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
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

  void evaluateHessians(const Pose3& pose, std::vector<Matrix>& G11) const {
    Matrix G11all = Z_6x6;
    Vector rT1 = pose.rotation().matrix().row(0);
    G11all.block<3,3>(3,0) = skewSymmetric(rT1);
    G11.push_back(G11all);
  }

  Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const {
    if (H)
      *H = (Matrix(1,6) << zeros(1,3), pose.rotation().matrix().row(0)).finished();
    return (Vector(1) << pose.x()).finished();
  }
};

TEST(testSQPSimple, poseOnALine) {
  const Key dualKey = 0;


  //Instantiate NLP
  NLP nlp;
  nlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0, 0)), noiseModel::Unit::Create(6)));
  LineConstraintX constraint(X(1), dualKey);
  nlp.nonlinearEqualities.add(constraint);

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(0.3, 0.2, 0.3), Point3(1,0,0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3()));

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
  Values actualSolution = sqpSimple.optimize(initialValues).first;

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

TEST(testSQPSimple, inequalityConstraint) {
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

  //Instantiate NLP
  NLP nlp;
  Values linPoint;
  linPoint.insert<Vector1>(X(1), zero(1));
  linPoint.insert<Vector1>(Y(1), zero(1));
  nlp.cost.add(LinearContainerFactor(hf, linPoint)); // wrap it using linearcontainerfactor
  nlp.linearInequalities.add(InequalityProblem1(X(1), Y(1), dualKey));

  Values initialValues;
  initialValues.insert(X(1), 1.0);
  initialValues.insert(Y(1), -10.0);

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
  Values actualValues = sqpSimple.optimize(initialValues).first;

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

TEST(testSQPSimple, poseWithABoundary) {
  const Key dualKey = 0;

  //Instantiate NLP
  NLP nlp;
  nlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0, 0)), noiseModel::Unit::Create(6)));
  AxisUpperBound constraint(X(1), X_AXIS, 0, dualKey);
  nlp.linearInequalities.add(constraint);

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(0.3, 0.2, 0.3), Point3(1, 0, 0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(0, 0, 0)));

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
  Values actualSolution = sqpSimple.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
}

TEST(testSQPSimple, poseWithinA2DBox) {
  const Key dualKey = 0;

  //Instantiate NLP
  NLP nlp;
  nlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(10, 0.5, 0)), noiseModel::Unit::Create(6)));
  nlp.linearInequalities.add(AxisLowerBound(X(1), X_AXIS, -1, dualKey));
  nlp.linearInequalities.add(AxisUpperBound(X(1), X_AXIS, 1, dualKey));
  nlp.linearInequalities.add(AxisLowerBound(X(1), Y_AXIS, -1, dualKey));
  nlp.linearInequalities.add(AxisUpperBound(X(1), Y_AXIS, 1, dualKey));

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(0.3, 0.2, 0.3), Point3(1, 0, 0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 0.5, 0)));

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
  Values actualSolution = sqpSimple.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
