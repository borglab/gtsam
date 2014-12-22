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
 * @author  Krunal Chande
 * @author  Duy-Nguyen Ta
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

namespace gtsam {

class LinearEqualityManifoldFactorGraph: public FactorGraph<NonlinearFactor> {
public:
  /// default constructor
  LinearEqualityManifoldFactorGraph() {
  }

  /// linearize to a LinearEqualityFactorGraph
  LinearEqualityFactorGraph::shared_ptr linearize(
      const Values& linearizationPoint) const {
    LinearEqualityFactorGraph::shared_ptr linearGraph(
        new LinearEqualityFactorGraph());
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
      JacobianFactor::shared_ptr jacobian = boost::dynamic_pointer_cast<JacobianFactor>(
          factor->linearize(linearizationPoint));
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      linearGraph->add(LinearEquality(*jacobian, constraint->dualKey()));
    }
    return linearGraph;
  }

  /**
   * Return true if the max absolute error all factors is less than a tolerance
   */
  bool checkFeasibility(const Values& values, double tol) const {
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
      NoiseModelFactor::shared_ptr noiseModelFactor = boost::dynamic_pointer_cast<NoiseModelFactor>(
          factor);
      Vector error = noiseModelFactor->unwhitenedError(values);
      if (error.lpNorm<Eigen::Infinity>() > tol) {
        return false;
      }
    }
    return true;
  }
};

class NonlinearEqualityFactorGraph: public LinearEqualityManifoldFactorGraph {
public:
  /// default constructor
  NonlinearEqualityFactorGraph() {
  }

  GaussianFactorGraph::shared_ptr multipliedHessians(const Values& values, const VectorValues& duals) const {
    GaussianFactorGraph::shared_ptr constrainedHessians(new GaussianFactorGraph());
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this) {
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      constrainedHessians->push_back(constraint->multipliedHessian(values, duals));
    }
    return constrainedHessians;
  }
};

class NonlinearInequalityFactorGraph : public FactorGraph<NonlinearFactor> {

public:
  /// default constructor
  NonlinearInequalityFactorGraph() {
  }

  /// linearize to a LinearEqualityFactorGraph
  LinearInequalityFactorGraph::shared_ptr linearize(
      const Values& linearizationPoint) const {
    LinearInequalityFactorGraph::shared_ptr linearGraph(
        new LinearInequalityFactorGraph());
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
      JacobianFactor::shared_ptr jacobian = boost::dynamic_pointer_cast<JacobianFactor>(
          factor->linearize(linearizationPoint));
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      linearGraph->add(LinearInequality(*jacobian, constraint->dualKey()));
    }
    return linearGraph;
  }

  /**
   * Return true if the error is <= 0.0
   */
  bool checkFeasibility(const Values& values, double tol) const {
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
      NoiseModelFactor::shared_ptr noiseModelFactor = boost::dynamic_pointer_cast<NoiseModelFactor>(
          factor);
      Vector error = noiseModelFactor->unwhitenedError(values);
      // TODO: Do we need to check if it's active or not?
      if (error[0] > tol) {
        return false;
      }
    }
    return true;
  }

  /**
   * Return true if the max absolute error all factors is less than a tolerance
   */
  bool checkDualFeasibility(const VectorValues& duals, double tol) const {
    BOOST_FOREACH(const Vector& dual, duals){
      if (dual[0] < 0.0) {
        return false;
      }
    }
    return true;
  }

  /**
   * Return true if the max absolute error all factors is less than a tolerance
   */
  bool checkComplimentaryCondition(const Values& values, const VectorValues& duals, double tol) const {
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
      NoiseModelFactor::shared_ptr noiseModelFactor = boost::dynamic_pointer_cast<NoiseModelFactor>(
          factor);
      Vector error = noiseModelFactor->unwhitenedError(values);
      if (error[0] > 0.0) {
        return false;
      }
    }
    return true;
  }
};

struct NLP {
  NonlinearFactorGraph cost;
  NonlinearEqualityFactorGraph linearEqualities;
  NonlinearEqualityFactorGraph nonlinearEqualities;
  NonlinearInequalityFactorGraph linearInequalities;
};

struct SQPSimpleState {
  Values values;
  VectorValues duals;
  bool converged;
  size_t iterations;

  /// Default constructor
  SQPSimpleState() : values(), duals(), converged(false), iterations(0) {}

  /// Constructor with an initialValues
  SQPSimpleState(const Values& initialValues) :
      values(initialValues), duals(VectorValues()), converged(false), iterations(0) {
  }
};

/**
 * Simple SQP optimizer to solve nonlinear constrained problems.
 * This simple version won't care about nonconvexity, which needs
 * more advanced techniques to solve, e.g., merit function, line search, second-order correction etc.
 */
class SQPSimple {
  NLP nlp_;
  static const double errorTol = 1e-5;
public:
  SQPSimple(const NLP& nlp) :
      nlp_(nlp) {
  }

  /// Check if \nabla f(x) - \lambda * \nabla c(x) == 0
  bool isDualFeasible(const VectorValues& delta) const {
    return delta.vector().lpNorm<Eigen::Infinity>() < errorTol
        && nlp_.linearInequalities.checkDualFeasibility(errorTol);
//    return false;
  }

  /// Check if c(x) == 0
  bool isPrimalFeasible(const SQPSimpleState& state) const {
    return nlp_.linearEqualities.checkFeasibility(state.values, errorTol)
        && nlp_.nonlinearEqualities.checkFeasibility(state.values, errorTol)
        && nlp_.linearInequalities.checkFeasibility(state.values, errorTol);
  }

  /// Check convergence
  bool checkConvergence(const SQPSimpleState& state, const VectorValues& delta) const {
    return isPrimalFeasible(state) & isDualFeasible(delta);
  }

  /**
   * Single iteration of SQP
   */
  SQPSimpleState iterate(const SQPSimpleState& state) const {
    static const bool debug = true;

    // construct the qp subproblem
    QP qp;
    qp.cost = *nlp_.cost.linearize(state.values);
    GaussianFactorGraph::shared_ptr multipliedHessians = nlp_.nonlinearEqualities.multipliedHessians(state.values, state.duals);
    qp.cost.push_back(*multipliedHessians);

    qp.equalities.add(*nlp_.linearEqualities.linearize(state.values));
    qp.equalities.add(*nlp_.nonlinearEqualities.linearize(state.values));

    qp.inequalities.add(*nlp_.linearInequalities.linearize(state.values));

    if (debug)
      qp.print("QP subproblem:");

    // solve the QP subproblem
    VectorValues delta, duals;
    QPSolver qpSolver(qp);
    boost::tie(delta, duals) = qpSolver.optimize();

    if (debug)
      delta.print("delta = ");

    if (debug)
      duals.print("duals = ");

    // update new state
    SQPSimpleState newState;
    newState.values = state.values.retract(delta);
    newState.duals = duals;
    newState.converged = checkConvergence(newState, delta);
    newState.iterations = state.iterations + 1;

    return newState;
  }

  VectorValues initializeDuals() const {
    VectorValues duals;
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.linearEqualities) {
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      duals.insert(constraint->dualKey(), zero(factor->dim()));
    }

    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.nonlinearEqualities) {
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      duals.insert(constraint->dualKey(), zero(factor->dim()));
    }
    return duals;
  }

  /**
   * Main optimization function.
   */
  std::pair<Values, VectorValues> optimize(const Values& initialValues) const {
    SQPSimpleState state(initialValues);
    state.duals = initializeDuals();

    while (!state.converged && state.iterations < 100) {
      state = iterate(state);
    }

    return std::make_pair(state.values, state.duals);
  }


};
}

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/nonlinear/NonlinearInequality.h>

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
  actualSolution.print("actualSolution: ");
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
  actualSolution.print("actualSolution: ");
  Pose3 pose(Rot3::ypr(0.1, 0.2, 0.3), Point3());
  Matrix hessian = numericalHessian<Pose3>(boost::bind(&LineConstraintX::computeError, constraint, _1), pose, 1e-2);
  cout << "hessian: \n" << hessian << endl;
}


//******************************************************************************
/**
 * Inequality boundary constraint
 *      x <= bound
 */
class UpperBoundX : public NonlinearInequality1<Pose3> {
  typedef NonlinearInequality1<Pose3> Base;
  double bound_;
public:
  UpperBoundX(Key key, double bound, Key dualKey) : Base(key, dualKey, 1), bound_(bound) {
  }

  double computeError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const {
    if (H)
      *H = (Matrix(1,6) << zeros(1,3), pose.rotation().matrix().row(0)).finished();
    return pose.x() - bound_;
  }
};

TEST(testSQPSimple, poseOnALine) {
  const Key dualKey = 0;


  //Instantiate NLP
  NLP nlp;
  nlp.cost.add(PriorFactor<Pose3>(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3(-1, 0, 0)), noiseModel::Unit::Create(6)));
  UpperBoundX constraint(X(1), 0, dualKey);
  nlp.nonlinearInequalities.add(constraint);

  Values initialValues;
  initialValues.insert(X(1), Pose3(Rot3::ypr(0.3, 0.2, 0.3), Point3(-1,0,0)));

  Values expectedSolution;
  expectedSolution.insert(X(1), Pose3(Rot3::ypr(0.1, 0.2, 0.3), Point3()));

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
  Values actualSolution = sqpSimple.optimize(initialValues).first;

  CHECK(assert_equal(expectedSolution, actualSolution, 1e-10));
  actualSolution.print("actualSolution: ");
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
