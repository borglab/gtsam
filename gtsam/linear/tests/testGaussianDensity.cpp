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
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

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
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
