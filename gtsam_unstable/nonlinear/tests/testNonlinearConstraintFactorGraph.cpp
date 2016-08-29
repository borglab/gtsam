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

#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityConstraint.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityFactorGraph.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam::symbol_shorthand;
using namespace gtsam;
const double tol = 1e-10;

/**
 * e^x - x^2 = 0
 */
struct Constraint1Error {
  Vector operator() (const double &x, boost::optional<Matrix &> H1 = boost::none) const {
    return I_1x1 * (exp(x) - std::pow(x, 2));
  }
};

/**
 * sin(x) = 0
 */
struct Constraint2Error {
  Vector operator() (const double &x, boost::optional<Matrix &> H1 = boost::none) const {
    return I_1x1 * (sin(x));
  }
};


typedef NonlinearEqualityConstraint1<double, Constraint1Error> Constraint1;
typedef NonlinearEqualityConstraint1<double, Constraint2Error> Constraint2;

//******************************************************************************
TEST(NonlinearInequalityFactorGraph, constructor) {
  NonlinearInequalityFactorGraph nonlinearInequalities;
  CHECK(nonlinearInequalities.empty());
}

TEST(NonlinearEqualityFactorGraph, equationSolvingWorks) {
  
  NonlinearEqualityFactorGraph solver;
  Key X1(Symbol('X',1)), X2(Symbol('X', 2));
  Constraint1 c1(X1, X2);
  Constraint2 c2(X1, X2);
  solver.push_back(c1);
  solver.push_back(c2);
}

TEST(NonlinearEqualityFactorGraph, errorCalculation){
  Key X1(Symbol('X',1)), lambda(Symbol('L',1));
  NonlinearEqualityFactorGraph solver;
  solver.push_back(Constraint1(X1,lambda));
  solver.push_back(Constraint2(X1,lambda));
  Values sample;
  sample.insert(X1, 0.0);
  Values sample1;
  sample1.insert(X1, 0.5);
  double actualErr = solver.cost(sample);
  double actualErr1 = solver.cost(sample1);
  double expectedError(1.0);
  double expectedError1(1.878);
  CHECK(assert_equal(expectedError, actualErr));
  CHECK(assert_equal(expectedError1, actualErr1, 1e-3));
}

TEST(NonlinearInequalityFactorGraph, errorCalculation){
  
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
