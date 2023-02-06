/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRot3.cpp
 * @brief   Unit tests for Rot3Q class
 * @author  Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

typedef BetweenFactor<Rot3> Between;
typedef NonlinearFactorGraph Graph;

/* ************************************************************************* */
TEST(Rot3, optimize) {

  // Optimize a circle
  Values truth;
  Values initial;
  Graph fg;
  fg.addPrior(Symbol('r',0), Rot3(), noiseModel::Isotropic::Sigma(3, 0.01));
  for(int j=0; j<6; ++j) {
    truth.insert(Symbol('r',j), Rot3::Rz(M_PI/3.0 * double(j)));
    initial.insert(Symbol('r',j), Rot3::Rz(M_PI/3.0 * double(j) + 0.1 * double(j%2)));
    fg += Between(Symbol('r',j), Symbol('r',(j+1)%6), Rot3::Rz(M_PI/3.0), noiseModel::Isotropic::Sigma(3, 0.01));
  }

  Values final = GaussNewtonOptimizer(fg, initial).optimize();

  EXPECT(assert_equal(truth, final, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
