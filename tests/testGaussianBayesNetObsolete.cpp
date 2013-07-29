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
#include <gtsam/inference/BayesNetOrdered.h>
#include <gtsam/linear/GaussianBayesNetOrdered.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <tests/smallExample.h>

using namespace std;
using namespace gtsam;
using namespace example;

static const Index _x_=0, _y_=1, _z_=2;

/* ************************************************************************* */
TEST( GaussianBayesNetOrdered, constructor )
{
  // small Bayes Net x <- y
  // x y d
  // 1 1 9
  //   1 5
  Matrix R11 = Matrix_(1,1,1.0), S12 = Matrix_(1,1,1.0);
  Matrix                         R22 = Matrix_(1,1,1.0);
  Vector d1(1), d2(1);
  d1(0) = 9; d2(0) = 5;
  Vector sigmas(1);
  sigmas(0) = 1.;

  // define nodes and specify in reverse topological sort (i.e. parents last)
  GaussianConditionalOrdered x(_x_,d1,R11,_y_,S12, sigmas), y(_y_,d2,R22, sigmas);

  // check small example which uses constructor
  GaussianBayesNetOrdered cbn = createSmallGaussianBayesNet();
  EXPECT( x.equals(*cbn[_x_]) );
  EXPECT( y.equals(*cbn[_y_]) );
}

/* ************************************************************************* */
TEST( GaussianBayesNetOrdered, matrix )
{
  // Create a test graph
  GaussianBayesNetOrdered cbn = createSmallGaussianBayesNet();

  Matrix R; Vector d;
  boost::tie(R,d) = matrix(cbn); // find matrix and RHS

  Matrix R1 = Matrix_(2,2,
          1.0, 1.0,
          0.0, 1.0
    );
  Vector d1 = Vector_(2, 9.0, 5.0);

  EXPECT(assert_equal(R,R1));
  EXPECT(assert_equal(d,d1));
}

/* ************************************************************************* */
TEST( GaussianBayesNetOrdered, optimize )
{
  GaussianBayesNetOrdered cbn = createSmallGaussianBayesNet();
  VectorValuesOrdered actual = optimize(cbn);

  VectorValuesOrdered expected(vector<size_t>(2,1));
  expected[_x_] = Vector_(1,4.);
  expected[_y_] = Vector_(1,5.);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNetOrdered, optimize2 )
{

  // Create empty graph
  GaussianFactorGraphOrdered fg;
  SharedDiagonal noise = noiseModel::Unit::Create(1);

  fg.add(_y_, eye(1), 2*ones(1), noise);

  fg.add(_x_, eye(1),_y_, -eye(1), -ones(1), noise);

  fg.add(_y_, eye(1),_z_, -eye(1), -ones(1), noise);

  fg.add(_x_, -eye(1), _z_, eye(1), 2*ones(1), noise);

  VectorValuesOrdered actual = *GaussianSequentialSolver(fg).optimize();

  VectorValuesOrdered expected(vector<size_t>(3,1));
  expected[_x_] = Vector_(1,1.);
  expected[_y_] = Vector_(1,2.);
  expected[_z_] = Vector_(1,3.);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNetOrdered, optimize3 )
{
  // y=R*x, x=inv(R)*y
  // 9 = 1 1   4
  // 5     1   5
  // NOTE: we are supplying a new RHS here
  GaussianBayesNetOrdered cbn = createSmallGaussianBayesNet();

  VectorValuesOrdered expected(vector<size_t>(2,1)), x(vector<size_t>(2,1));
  expected[_x_] = Vector_(1, 4.);
  expected[_y_] = Vector_(1, 5.);

  // test functional version
  VectorValuesOrdered actual = optimize(cbn);
  EXPECT(assert_equal(expected,actual));

  // test imperative version
  optimizeInPlace(cbn,x);
  EXPECT(assert_equal(expected,x));
}

/* ************************************************************************* */
TEST( GaussianBayesNetOrdered, backSubstituteTranspose )
{
  // x=R'*y, y=inv(R')*x
  // 2 = 1    2
  // 5   1 1  3
  GaussianBayesNetOrdered cbn = createSmallGaussianBayesNet();

  VectorValuesOrdered y(vector<size_t>(2,1)), x(vector<size_t>(2,1));
  x[_x_] = Vector_(1,2.);
  x[_y_] = Vector_(1,5.);
  y[_x_] = Vector_(1,2.);
  y[_y_] = Vector_(1,3.);

  // test functional version
  VectorValuesOrdered actual = backSubstituteTranspose(cbn,x);
  EXPECT(assert_equal(y,actual));
}

/* ************************************************************************* */
// Tests computing Determinant
TEST( GaussianBayesNetOrdered, DeterminantTest )
{
  GaussianBayesNetOrdered cbn;
  cbn += boost::shared_ptr<GaussianConditionalOrdered>(new GaussianConditionalOrdered(
          0, Vector_( 2, 3.0, 4.0 ), Matrix_(2, 2, 1.0, 3.0, 0.0, 4.0 ),
          1, Matrix_(2, 2, 2.0, 1.0, 2.0, 3.0),
          ones(2)));

  cbn += boost::shared_ptr<GaussianConditionalOrdered>(new GaussianConditionalOrdered(
          1, Vector_( 2, 5.0, 6.0 ), Matrix_(2, 2, 1.0, 1.0, 0.0, 3.0 ),
          2, Matrix_(2, 2, 1.0, 0.0, 5.0, 2.0),
          ones(2)));

  cbn += boost::shared_ptr<GaussianConditionalOrdered>(new GaussianConditionalOrdered(
      3, Vector_( 2, 7.0, 8.0 ), Matrix_(2, 2, 1.0, 1.0, 0.0, 5.0 ),
      ones(2)));

  double expectedDeterminant = 60;
  double actualDeterminant = determinant(cbn);

  EXPECT_DOUBLES_EQUAL( expectedDeterminant, actualDeterminant, 1e-9);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
