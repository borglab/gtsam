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
#include <boost/math/constants/constants.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

typedef TypedSymbol<Rot3, 'r'> Key;
typedef PriorFactor<Key> Prior;
typedef BetweenFactor<Key> Between;
typedef NonlinearFactorGraph Graph;

/* ************************************************************************* */
TEST(Rot3, optimize) {

  // Optimize a circle
  DynamicValues truth;
  DynamicValues initial;
  Graph fg;
  fg.add(Prior(0, Rot3(), sharedSigma(3, 0.01)));
  for(int j=0; j<6; ++j) {
    truth.insert(Key(j), Rot3::Rz(M_PI/3.0 * double(j)));
    initial.insert(Key(j), Rot3::Rz(M_PI/3.0 * double(j) + 0.1 * double(j%2)));
    fg.add(Between(Key(j), Key((j+1)%6), Rot3::Rz(M_PI/3.0), sharedSigma(3, 0.01)));
  }

  NonlinearOptimizationParameters params;
  DynamicValues final = optimize(fg, initial, params);

  EXPECT(assert_equal(truth, final, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

