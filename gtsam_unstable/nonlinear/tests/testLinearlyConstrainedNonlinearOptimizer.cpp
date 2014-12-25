/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testLinearlyConstrainedNonlinearOptimizer.cpp
 * @brief   Unit tests for LinearlyConstrainedNonlinearOptimizer
 * @author  Krunal Chande
 * @author  Duy-Nguyen Ta
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>


//namespace gtsam {
//struct LinearlyConstrainedNLP {
//  NonlinearFactorGraph cost;
//  LinearEqualityFactorGraph equalities;
//  LinearInequalityFactorGraph inequalities;
//};
//
//struct LinearlyConstrainedNLPState {
//  Values values;
//  VectorValues duals;
//  bool converged;
//  LinearlyConstrainedNLPState(const Values& initialValues) :
//      values(initialValues), duals(VectorValues()), converged(false) {
//  }
//};
//class LinearlyConstrainedNonLinearOptimizer {
//  LinearlyConstrainedNLP lcNLP_;
//public:
//  LinearlyConstrainedNonLinearOptimizer(const LinearlyConstrainedNLP& lcNLP): lcNLP_(lcNLP) {}
//
//  LinearlyConstrainedNLPState iterate(const LinearlyConstrainedNLPState& state) const {
//    QP qp;
//    qp.cost = lcNLP_.cost.linearize(state.values);
//    qp.equalities = lcNLP_.equalities;
//    qp.inequalities = lcNLP_.inequalities;
//    QPSolver qpSolver(qp);
//    VectorValues delta, duals;
//    boost::tie(delta, duals) = qpSolver.optimize();
//    LinearlyConstrainedNLPState newState;
//    newState.values = state.values.retract(delta);
//    newState.duals = duals;
//    newState.converged = checkConvergence(newState.values, newState.duals);
//    return newState;
//  }
//
//  /**
//   * Main optimization function.
//   */
//  std::pair<Values, VectorValues> optimize(const Values& initialValues) const {
//    LinearlyConstrainedNLPState state(initialValues);
//    while(!state.converged){
//      state = iterate(state);
//    }
//
//    return std::make_pair(initialValues, VectorValues());
//  }
//};
//}
//
//using namespace std;
//using namespace gtsam::symbol_shorthand;
//using namespace gtsam;
//const double tol = 1e-10;
////******************************************************************************
//TEST(LinearlyConstrainedNonlinearOptimizer, Problem1 ) {
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
//  inequalities.push_back(LinearInequality(X(1), ones(1,1), X(2), ones(1,1), 2, 0)); // x1 + x2 <= 2 --> x1 + x2 -2 <= 0, --> b=2
//  inequalities.push_back(LinearInequality(X(1), -ones(1,1), 0, 1));                 // -x1     <= 0
//  inequalities.push_back(LinearInequality(X(2), -ones(1,1), 0, 2));                 //    -x2  <= 0
//  inequalities.push_back(LinearInequality(X(1), ones(1,1), 1.5, 3));                // x1      <= 3/2
//
//  // Instantiate LinearlyConstrainedNLP, pretending that the cost is not quadratic
//  // (LinearContainerFactor makes a linear factor behave like a nonlinear one)
//  LinearlyConstrainedNLP lcNLP;
//  lcNLP.cost.add(LinearContainerFactor(lf));
//  lcNLP.inequalities = inequalities;
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
//  // instantiate LinearlyConstrainedNonLinearOptimizer
//  LinearlyConstrainedNonLinearOptimizer lcNLPSolver(lcNLP);
//  // create initial values for optimization
//  Values initialValues;
//  initialValues.insert(X(1), 0.0);
//  initialValues.insert(X(2), 0.0);
//  Values actualSolution = lcNLPSolver.optimize(initialValues).first;
//
//
//  DOUBLES_EQUAL(expectedSolution.at(X(1))[0], actualSolution.at<double>(X(1)), tol);
//  DOUBLES_EQUAL(expectedSolution.at(X(2))[0], actualSolution.at<double>(X(2)), tol);
//}
//
//******************************************************************************
int main() {
  std::cout<<"here"<<std::endl;
//  TestResult tr;
//  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
//
