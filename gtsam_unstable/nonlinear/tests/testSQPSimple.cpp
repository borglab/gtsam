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


struct NLP {
  NonlinearFactorGraph cost;
  NonlinearEqualityFactorGraph linearEqualities;
  NonlinearEqualityFactorGraph nonlinearEqualities;
};

struct SQPSimpleState {
  Values values;
  VectorValues duals;
  bool converged;

  /// Default constructor
  SQPSimpleState() : values(), duals(), converged(false) {}

  /// Constructor with an initialValues
  SQPSimpleState(const Values& initialValues) :
      values(initialValues), duals(VectorValues()), converged(false) {
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
    return delta.vector().lpNorm<Eigen::Infinity>() < errorTol;
  }

  /// Check if c(x) == 0
  bool isPrimalFeasible(const SQPSimpleState& state) const {
    return nlp_.linearEqualities.checkFeasibility(state.values, errorTol)
        && nlp_.nonlinearEqualities.checkFeasibility(state.values, errorTol);
  }

  /// Check convergence
  bool checkConvergence(const SQPSimpleState& state, const VectorValues& delta) const {
    return isPrimalFeasible(state) & isDualFeasible(delta);
  }

  /**
   * Single iteration of SQP
   */
  SQPSimpleState iterate(const SQPSimpleState& state) const {
    // construct the qp subproblem
    QP qp;
    qp.cost = *nlp_.cost.linearize(state.values);
    GaussianFactorGraph::shared_ptr multipliedHessians = nlp_.nonlinearEqualities.multipliedHessians(state.values, state.duals);
    qp.cost.push_back(*multipliedHessians);

    qp.equalities.add(*nlp_.linearEqualities.linearize(state.values));
    qp.equalities.add(*nlp_.nonlinearEqualities.linearize(state.values));

    // solve the QP subproblem
    VectorValues delta, duals;
    QPSolver qpSolver(qp);
    boost::tie(delta, duals) = qpSolver.optimize();

    // update new state
    SQPSimpleState newState;
    newState.values = state.values.retract(delta);
    newState.duals = duals;
    newState.converged = checkConvergence(newState, delta);

    return newState;
  }

  /**
   * Main optimization function.
   */
  std::pair<Values, VectorValues> optimize(const Values& initialValues) const {
    SQPSimpleState state(initialValues);
    while (!state.converged) {
      state = iterate(state);
    }

    return std::make_pair(initialValues, VectorValues());
  }


};
}

using namespace std;
using namespace gtsam::symbol_shorthand;
using namespace gtsam;
const double tol = 1e-10;
//******************************************************************************
TEST(testSQPSimple, Problem2) {
  // Simple quadratic cost: x1^2 + x2^2
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence here we have G11 = 2, G12 = 0, G22 = 2, g1 = 0, g2 = 0, f = 0
  HessianFactor hf(X(1), X(2), 2.0 * ones(1,1), zero(1), zero(1),
                    2*ones(1,1), zero(1) , 0);

  LinearEqualityFactorGraph equalities;
  equalities.push_back(LinearEquality(X(1), ones(1), X(2), ones(1), -1*ones(1), 0)); // x + y - 1 = 0

  // Compare against QP
  QP qp;
  qp.cost.add(hf);
  qp.equalities = equalities;

  // instantiate QPsolver
  QPSolver qpSolver(qp);
  // create initial values for optimization
  VectorValues initialVectorValues;
  initialVectorValues.insert(X(1), zero(1));
  initialVectorValues.insert(X(2), zero(1));
  VectorValues expectedSolution = qpSolver.optimize(initialVectorValues).first;
  cout<<"expectedSolution.at(X(1))[0]: "<<expectedSolution.at(X(1))[0]<<endl;
  cout<<"expectedSolution.at(X(2))[0]: "<<expectedSolution.at(X(2))[0]<<endl;

  //Instantiate NLP
  NLP nlp;
  nlp.cost.add(); // wrap it using linearcontainerfactor
  nlp.linearEqualities // for constraint it has to inherit from
  // write an evaluate error and return jacobian

  // Instantiate SQP
  SQPSimple sqpSimple(nlp);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

//TEST(testSQPSimple, Problem1 ) {
//
//  // build a quadratic Objective function x1^2 - x1*x2 + x2^2 - 3*x1 + 5
//  // Note the Hessian encodes:
//  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
//  // Hence, we have G11=2, G12 = -1, g1 = +3, G22 = 2, g2 = 0, f = 10
//  HessianFactor lf(X(1), X(2), 2.0 * ones(1, 1), -ones(1, 1), 3.0 * ones(1),
//      2.0 * ones(1, 1), zero(1), 10.0);
//
//  // build linear inequalities
//  LinearInequalityFactorGraph inequalities;
//  inequalities.push_back(
//      LinearInequality(X(1), ones(1, 1), X(2), ones(1, 1), 2, 0)); // x1 + x2 <= 2 --> x1 + x2 -2 <= 0, --> b=2
//  inequalities.push_back(LinearInequality(X(1), -ones(1, 1), 0, 1)); // -x1     <= 0
//  inequalities.push_back(LinearInequality(X(2), -ones(1, 1), 0, 2)); //    -x2  <= 0
//  inequalities.push_back(LinearInequality(X(1), ones(1, 1), 1.5, 3)); // x1      <= 3/2
//
//  // Compare against a QP
//  QP qp;
//  qp.cost.add(lf);
//  qp.inequalities = inequalities;
//
//  // instantiate QPsolver
//  QPSolver qpSolver(qp);
//  // create initial values for optimization
//  VectorValues initialVectorValues;
//  initialVectorValues.insert(X(1), zero(1));
//  initialVectorValues.insert(X(2), zero(1));
//  VectorValues expectedSolution = qpSolver.optimize(initialVectorValues).first;
//
//
//  NonlinearEqualityFactorGraph linearEqualities;
//  NonlinearEqualityFactorGraph nonlinearEqualities;
//  nonlinearEqualities.push_back(NonlinearEquality(X(1), ones(1, 1), X(2), ones(1, 1), 2, 0)));
//
//  NLP nlp;
//  nlp.cost.add(lf);
//  nlp.linearEqualities.push_back(NonlinearEqualityFactorGraph());
//  // instantiate QPsolver
//  SQPSimple sqpSolver(nlp);
//  // create initial values for optimization
//  Values initialValues;
//  initialValues.insert(X(1), zero(1));
//  initialValues.insert(X(2), zero(1));
//
//  std::pair<Vector, VectorValues>  actualSolution = sqpSolver.optimize(initialValues);
//
//  DOUBLES_EQUAL(expectedSolution.at(X(1))[0], actualSolution.at<double>(X(1)),
//      tol);
//  DOUBLES_EQUAL(expectedSolution.at(X(2))[0], actualSolution.at<double>(X(2)),
//      tol);
//}
