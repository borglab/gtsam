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

TEST_DISABLED(HessianFactor, ExampleThatShouldWork){
  GaussianFactorGraph fg1;
  //THIS SHOULD EQUAL x1^2 + x2^2 + x1x2 - x1 - x2 + 10
  // In this case the hessian is indefinite(one negative and one positive eigen vale.
  // Still the problem has a solution
  HessianFactor factor2d(X(1), X(2), 2*I_1x1, 2*I_1x1, I_1x1, 2*I_1x1, I_1x1, 10);
  GTSAM_PRINT(factor2d);
  fg1.push_back(factor2d);
  fg1.optimize();
}

TEST(HessianFactor, TestFailsOnPostiveDefinite){
  GaussianFactorGraph fg1;
  //THIS SHOULD EQUAL x1^2 + x2^2 + x1x2 - x1 - x2 + 10
  // THE HESSIAN IS POSITIVE DEFINITE AND IT HAS A UNIQUE MINIMUM
  HessianFactor factor2d(X(1), X(2), 8*I_1x1, 2*I_1x1, -1.5*I_1x1, 10*I_1x1, 2*I_1x1, 0);
  GTSAM_PRINT(factor2d);
  fg1.push_back(factor2d);
  fg1.optimize();
}
TEST(HessianFactor, TestSuccedsOnPostiveDefinite){
  GaussianFactorGraph fg1;
  //THIS SHOULD EQUAL x1^2 + x2^2 + x1x2 - x1 - x2 + 10
  // THE HESSIAN IS POSITIVE DEFINITE AND IT HAS A UNIQUE MINIMUM
  HessianFactor factor2d(X(1), X(2), 8*I_1x1, 2*I_1x1, -1.5*I_1x1, 10*I_1x1, 2*I_1x1, 100);
  GTSAM_PRINT(factor2d);
  fg1.push_back(factor2d);
  fg1.optimize();
}

TEST(HessianFactor, TestFailsOnPositiveDefine1) {
  GaussianFactorGraph fg;
  const HessianFactor &factor = HessianFactor(X(1), 2*I_1x1, -I_1x1, 2*-20);
  GTSAM_PRINT(factor);
  fg.push_back(factor);
  VectorValues solution  = fg.optimize();
  CHECK(assert_equal(-0.5*I_1x1, solution.at(X(1)), 1e-9));
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */