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

class Constraint1: public NonlinearEqualityConstraint1<double> {
public:
  Constraint1(Key key, Key dualKey) :
      NonlinearEqualityConstraint1(key, dualKey, 1) {
  }

  Vector evaluateError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return Vector1(exp(x) - std::pow(x, 2));
  }
};

class Constraint2: public NonlinearEqualityConstraint1<double> {
public:
  Constraint2(Key key, Key dualKey) :
      NonlinearEqualityConstraint1(key, dualKey, 1) {
  }

  Vector evaluateError(const X &x, boost::optional<Matrix &> H1) const
      override {
    return Vector1(sin(x));
  }
};

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

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
