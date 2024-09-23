/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianDensity.cpp
 *  @brief  Unit tests for Gaussian Density
 *  @author Frank Dellaert
 *  @date   Jan 21, 2012
 **/

#include <gtsam/linear/GaussianDensity.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;
using symbol_shorthand::X;

/* ************************************************************************* */
TEST(GaussianDensity, constructor)
{
  Matrix R = (Matrix(2,2) <<
      -12.1244,  -5.1962,
            0.,   4.6904).finished();

  Vector d = Vector2(1.0, 2.0), s = Vector2(3.0, 4.0);
  GaussianConditional conditional(1, d, R, noiseModel::Diagonal::Sigmas(s));

  GaussianDensity copied(conditional);
  EXPECT(assert_equal(d, copied.d()));
  EXPECT(assert_equal(s, copied.get_model()->sigmas()));
}

/* ************************************************************************* */
// Test FromMeanAndStddev named constructor
TEST(GaussianDensity, FromMeanAndStddev) {
  Matrix A1 = (Matrix(2, 2) << 1., 2., 3., 4.).finished();
  const Vector2 b(20, 40), x0(1, 2);
  const double sigma = 3;

  VectorValues values;
  values.insert(X(0), x0);

  auto density = GaussianDensity::FromMeanAndStddev(X(0), b, sigma);
  Vector2 e = (x0 - b) / sigma;
  double expected1 = 0.5 * e.dot(e);
  EXPECT_DOUBLES_EQUAL(expected1, density.error(values), 1e-9);

  double expected2 = -(density.errorConstant() + 0.5 * e.dot(e));
  EXPECT_DOUBLES_EQUAL(expected2, density.logProbability(values), 1e-9);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
