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
#include <gtsam/geometry/Rot3Q.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearOptimization-inl.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

typedef TypedSymbol<Rot3Q, 'r'> KeyQ;
typedef Values<KeyQ> ValuesQ;
typedef PriorFactor<ValuesQ, KeyQ> PriorQ;
typedef BetweenFactor<ValuesQ, KeyQ> BetweenQ;
typedef NonlinearFactorGraph<ValuesQ> GraphQ;

typedef TypedSymbol<Rot3M, 'r'> KeyM;
typedef Values<KeyM> ValuesM;
typedef PriorFactor<ValuesM, KeyM> PriorM;
typedef BetweenFactor<ValuesM, KeyM> BetweenM;
typedef NonlinearFactorGraph<ValuesM> GraphM;

/* ************************************************************************* */
TEST(Rot3Q, optimize1) {
  GraphQ fgQ;
  fgQ.add(PriorQ(0, Rot3Q(), sharedSigma(3, 0.01)));
  fgQ.add(BetweenQ(0, 1, Rot3Q::Rz(M_PI/3.0), sharedSigma(3, 0.01)));
  fgQ.add(BetweenQ(1, 0, Rot3Q::Rz(5.0*M_PI/3.0), sharedSigma(3, 0.01)));

  GraphM fgM;
  fgM.add(PriorM(0, Rot3M(), sharedSigma(3, 0.01)));
  fgM.add(BetweenM(0, 1, Rot3M::Rz(M_PI/3.0), sharedSigma(3, 0.01)));
  fgM.add(BetweenM(1, 0, Rot3M::Rz(5.0*M_PI/3.0), sharedSigma(3, 0.01)));

  ValuesQ initialQ;
  initialQ.insert(0, Rot3Q::Rz(0.0));
  initialQ.insert(1, Rot3Q::Rz(M_PI/3.0 + 0.1));

  ValuesM initialM;
  initialM.insert(0, Rot3M::Rz(0.0));
  initialM.insert(1, Rot3M::Rz(M_PI/3.0 + 0.1));

  ValuesQ truthQ;
  truthQ.insert(0, Rot3Q::Rz(0.0));
  truthQ.insert(1, Rot3Q::Rz(M_PI/3.0));

  ValuesM truthM;
  truthM.insert(0, Rot3M::Rz(0.0));
  truthM.insert(1, Rot3M::Rz(M_PI/3.0));

  // Compare Matrix and Quaternion between
  Matrix H1M, H2M;
  Rot3M betwM = initialM[1].between(initialM[0], H1M, H2M);
  Matrix H1Q, H2Q;
  Rot3Q betwQ = initialM[1].between(initialM[0], H1Q, H2Q);
  EXPECT(assert_equal(betwM.matrix(), betwQ.matrix()));
  EXPECT(assert_equal(H1M, H1Q));
  EXPECT(assert_equal(H2M, H2Q));
  Point3 x1(1.0,0.0,0.0), x2(0.0,1.0,0.0);
  EXPECT(assert_equal(betwM*x1, betwQ*x1));
  EXPECT(assert_equal(betwM*x2, betwQ*x2));

  // Compare Matrix and Quaternion logmap
  Vector logM = initialM[1].localCoordinates(initialM[0]);
  Vector logQ = initialQ[1].localCoordinates(initialQ[0]);
  EXPECT(assert_equal(logM, logQ));

  // Compare Matrix and Quaternion linear graph
  Ordering ordering; ordering += KeyQ(0), KeyQ(1);
  GaussianFactorGraph gfgQ(*fgQ.linearize(initialQ, ordering));
  GaussianFactorGraph gfgM(*fgM.linearize(initialM, ordering));
  EXPECT(assert_equal(gfgQ, gfgM, 1e-5));

  NonlinearOptimizationParameters params;
  //params.verbosity_ = NonlinearOptimizationParameters::TRYLAMBDA;
  ValuesQ final = optimize(fgQ, initialQ, params);

  EXPECT(assert_equal(truthQ, final, 1e-5));
}

/* ************************************************************************* */
TEST(Rot3Q, optimize) {

  // Optimize a circle
  ValuesQ truth;
  ValuesQ initial;
  GraphQ fg;
  fg.add(PriorQ(0, Rot3Q(), sharedSigma(3, 0.01)));
  for(int j=0; j<6; ++j) {
    truth.insert(j, Rot3Q::Rz(M_PI/3.0 * double(j)));
    initial.insert(j, Rot3Q::Rz(M_PI/3.0 * double(j) + 0.1 * double(j%2)));
    fg.add(BetweenQ(j, (j+1)%6, Rot3Q::Rz(M_PI/3.0), sharedSigma(3, 0.01)));
  }

  NonlinearOptimizationParameters params;
  //params.verbosity_ = NonlinearOptimizationParameters::TRYLAMBDA;
  ValuesQ final = optimize(fg, initial, params);

  EXPECT(assert_equal(truth, final, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

