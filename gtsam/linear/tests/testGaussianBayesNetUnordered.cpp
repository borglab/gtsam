/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianBayesNet.cpp
 * @brief   Unit tests for GaussianBayesNet
 * @author  Frank Dellaert
 */

// STL/C++
#include <iostream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace std;
using namespace gtsam;

static const Key _x_=0, _y_=1, _z_=2;

static GaussianBayesNet smallBayesNet = list_of
  (GaussianConditional(_x_, Vector_(1, 9.0), Matrix_(1, 1, 1.0), _y_, Matrix_(1, 1, 1.0)))
  (GaussianConditional(_y_, Vector_(1, 5.0), Matrix_(1, 1, 1.0)));

/* ************************************************************************* */
TEST( GaussianBayesNet, matrix )
{
  Matrix R; Vector d;
  boost::tie(R,d) = smallBayesNet.matrix(); // find matrix and RHS

  Matrix R1 = Matrix_(2,2,
          1.0, 1.0,
          0.0, 1.0
    );
  Vector d1 = Vector_(2, 9.0, 5.0);

  EXPECT(assert_equal(R,R1));
  EXPECT(assert_equal(d,d1));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimize )
{
  VectorValues actual = smallBayesNet.optimize();

  VectorValues expected = map_list_of
    (_x_, Vector_(1, 4.0))
    (_y_, Vector_(1, 5.0));

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimize3 )
{
  // y = R*x, x=inv(R)*y
  // 4 = 1 1   -1 
  // 5     1    5
  // NOTE: we are supplying a new RHS here

  VectorValues expected = map_list_of
    (_x_, Vector_(1, -1.0))
    (_y_, Vector_(1,  5.0));

  // Test different RHS version
  VectorValues gx = map_list_of
    (_x_, Vector_(1, 4.0))
    (_y_, Vector_(1, 5.0));
  VectorValues actual = smallBayesNet.backSubstitute(gx);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, backSubstituteTranspose )
{
  // x=R'*y, expected=inv(R')*x
  // 2 = 1    2
  // 5   1 1  3
  VectorValues
    x = map_list_of
      (_x_, Vector_(1, 2.0))
      (_y_, Vector_(1, 5.0)),
    expected = map_list_of
      (_x_, Vector_(1, 2.0))
      (_y_, Vector_(1, 3.0));

  VectorValues actual = smallBayesNet.backSubstituteTranspose(x);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Tests computing Determinant
TEST( GaussianBayesNet, DeterminantTest )
{
  GaussianBayesNet cbn;
  cbn += GaussianConditional(
          0, Vector_( 2, 3.0, 4.0 ), Matrix_(2, 2, 1.0, 3.0, 0.0, 4.0 ),
          1, Matrix_(2, 2, 2.0, 1.0, 2.0, 3.0), noiseModel::Isotropic::Sigma(2, 2.0));

  cbn += GaussianConditional(
          1, Vector_( 2, 5.0, 6.0 ), Matrix_(2, 2, 1.0, 1.0, 0.0, 3.0 ),
          2, Matrix_(2, 2, 1.0, 0.0, 5.0, 2.0), noiseModel::Isotropic::Sigma(2, 2.0));

  cbn += GaussianConditional(
      3, Vector_( 2, 7.0, 8.0 ), Matrix_(2, 2, 1.0, 1.0, 0.0, 5.0 ), noiseModel::Isotropic::Sigma(2, 2.0));

  double expectedDeterminant = 60.0 / 64.0;
  double actualDeterminant = cbn.determinant();

  EXPECT_DOUBLES_EQUAL( expectedDeterminant, actualDeterminant, 1e-9);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
