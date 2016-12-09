/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file HessianErrorDemonstration.cpp
 * @brief A Simple Demonstration of GTSAM failing to solve a quadratic program with a positive definite Hessian
 * @date Nov 15, 2016
 * @author Ivan Dario Jimenez
 */

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/linear/QPSParser.h>
#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;
/*
 * HessianFactor implements a general quadratic factor of the form
 * \f[ E(x) = 0.5 x^T G x - x^T g + 0.5 f \f]
 * FROM THE HESSIAN FACTOR IMPLEMENTATION
 */

HessianFactor factor2d(X(1), X(2), 8*I_1x1, 2*I_1x1, -1.5*I_1x1, 10*I_1x1, 2*I_1x1, 0);
HessianFactor factor2dRaised(X(1), X(2), 8*I_1x1, 2*I_1x1, -1.5*I_1x1, 10*I_1x1, 2*I_1x1, 100);
HessianFactor factor2dRaisedMore(X(1), X(2), 8*I_1x1, 2*I_1x1, -1.5*I_1x1, 10*I_1x1, 2*I_1x1, 1000);
LinearEquality jacobianFactorWithNoiseModel(X(1), -2*I_1x1, X(2), -1*I_1x1, -2.0* I_1x1, 0);
JacobianFactor jacobianFactor(X(1), -2*I_1x1, X(2), -1*I_1x1, -2.0* I_1x1, 0);

TEST(HessianFactor, TestFailsOnPostiveDefinite){
  GaussianFactorGraph fg1;
  fg1.push_back(factor2d);
  fg1.push_back(jacobianFactorWithNoiseModel);
  VectorValues actual = fg1.optimize();
  VectorValues expected;
  expected.insert(X(1), 0.327586207*I_1x1);
  expected.insert(X(2), 0.099137931*I_1x1);
  CHECK(assert_equal(expected, actual, 1e-7))
}
TEST(HessianFactor, TestSuccedsOnPostiveDefinite){
  GaussianFactorGraph fg1;
  fg1.push_back(factor2d);
  fg1.push_back(jacobianFactor);
  VectorValues actual = fg1.optimize();
  VectorValues expected;
  expected.insert(X(2), 0.327586207*I_1x1);
  expected.insert(X(1), 0.099137931*I_1x1);
  CHECK(assert_equal(expected, actual, 1e-7))
}

TEST(HessianFactor, TestSuccedsWithAddedConstant){
  GaussianFactorGraph fg1;
  fg1.push_back(factor2dRaised);
  fg1.push_back(jacobianFactorWithNoiseModel);
  VectorValues actual = fg1.optimize();
  VectorValues expected;
  expected.insert(X(2), 0.327586207*I_1x1);
  expected.insert(X(1), 0.099137931*I_1x1);
  CHECK(assert_equal(expected, actual, 1e-7))
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */