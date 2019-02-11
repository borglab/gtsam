/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSimulated2DOriented.cpp
 * @date Jun 10, 2010
 * @author nikai
 * @brief unit tests for simulated2DOriented
 */

#include <tests/simulated2D.h>
#include <tests/simulated2DOriented.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;
using namespace gtsam;

#include <iostream>
#include <CppUnitLite/TestHarness.h>

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( simulated2DOriented, Dprior )
{
  Pose2 x(1,-9, 0.1);
  Matrix numerical = numericalDerivative11(simulated2DOriented::prior,x);
  Matrix computed;
  simulated2DOriented::prior(x,computed);
  CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
  TEST( simulated2DOriented, DOdo )
{
  Pose2 x1(1,-9,0.1),x2(-5,6,0.2);
  Matrix H1,H2;
  simulated2DOriented::odo(x1,x2,H1,H2);
  Matrix A1 = numericalDerivative21(simulated2DOriented::odo,x1,x2);
  CHECK(assert_equal(A1,H1,1e-9));
  Matrix A2 = numericalDerivative22(simulated2DOriented::odo,x1,x2);
  CHECK(assert_equal(A2,H2,1e-9));
}

/* ************************************************************************* */
TEST( simulated2DOriented, constructor )
{
  Pose2 measurement(0.2, 0.3, 0.1);
  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector3(1., 1., 1.));
  simulated2DOriented::Odometry factor(measurement, model, X(1), X(2));

  simulated2DOriented::Values config;
  config.insert(X(1), Pose2(1., 0., 0.2));
  config.insert(X(2), Pose2(2., 0., 0.1));
  std::shared_ptr<GaussianFactor> lf = factor.linearize(config);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
