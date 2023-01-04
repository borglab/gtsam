/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussMarkov1stOrderFactor.cpp
 * @brief   Unit tests for the GaussMarkov1stOrder factor
 * @author  Vadim Indelman
 * @date    Jan 17, 2012
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/slam/GaussMarkov1stOrderFactor.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

//! Factors
typedef GaussMarkov1stOrderFactor<Vector3> GaussMarkovFactor;

/* ************************************************************************* */
Vector predictionError(const Vector& v1, const Vector& v2,
                       const GaussMarkovFactor factor) {
  return factor.evaluateError(v1, v2);
}

/* ************************************************************************* */
TEST( GaussMarkovFactor, equals )
{
  // Create two identical factors and make sure they're equal
  Key x1(1);
  Key x2(2);
  double delta_t = 0.10;
  Vector tau = Vector3(100.0, 150.0, 10.0);
  SharedGaussian model = noiseModel::Isotropic::Sigma(3, 1.0);

  GaussMarkovFactor factor1(x1, x2, delta_t, tau, model);
  GaussMarkovFactor factor2(x1, x2, delta_t, tau, model);

  CHECK(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( GaussMarkovFactor, error )
{
  Values linPoint;
  Key x1(1);
  Key x2(2);
  double delta_t = 0.10;
  Vector3 tau(100.0, 150.0, 10.0);
  SharedGaussian model = noiseModel::Isotropic::Sigma(3, 1.0);

  Vector3 v1(10.0, 12.0, 13.0);
  Vector3 v2(10.0, 15.0, 14.0);

  // Create two nodes
  linPoint.insert(x1, v1);
  linPoint.insert(x2, v2);

  GaussMarkovFactor factor(x1, x2, delta_t, tau, model);
  Vector3 error1 = factor.evaluateError(v1, v2);

  // Manually calculate the error
  Vector3 alpha(tau.size());
  Vector3 alpha_v1(tau.size());
  for(int i=0; i<tau.size(); i++){
    alpha(i) = exp(- 1/tau(i)*delta_t );
    alpha_v1(i) = alpha(i) * v1(i);
  }
  Vector3 error2 = v2 - alpha_v1;

  CHECK(assert_equal(error1, error2, 1e-8));
}

/* ************************************************************************* */
TEST (GaussMarkovFactor, jacobian ) {

  Values linPoint;
  Key x1(1);
  Key x2(2);
  double delta_t = 0.10;
  Vector3 tau(100.0, 150.0, 10.0);
  SharedGaussian model = noiseModel::Isotropic::Sigma(3, 1.0);

  GaussMarkovFactor factor(x1, x2, delta_t, tau, model);

  // Update the linearization point
  Vector3 v1_upd(0.5, -0.7, 0.3);
  Vector3 v2_upd(-0.7, 0.4, 0.9);

  // Calculate the Jacobian matrix using the factor
  Matrix computed_H1, computed_H2;
  factor.evaluateError(v1_upd, v2_upd, computed_H1, computed_H2);

  // Calculate the Jacobian matrices H1 and H2 using the numerical derivative function
  Matrix numerical_H1, numerical_H2;
  numerical_H1 = numericalDerivative21<Vector3, Vector3, Vector3>(
      std::bind(&predictionError, std::placeholders::_1, std::placeholders::_2,
                factor),
      v1_upd, v2_upd);
  numerical_H2 = numericalDerivative22<Vector3, Vector3, Vector3>(
      std::bind(&predictionError, std::placeholders::_1, std::placeholders::_2,
                factor),
      v1_upd, v2_upd);

  // Verify they are equal for this choice of state
  CHECK(assert_equal(numerical_H1, computed_H1, 1e-9));
  CHECK(assert_equal(numerical_H2, computed_H2, 1e-9));
}

/* ************************************************************************* */
int main()
{
  TestResult tr; return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

