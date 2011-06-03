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

#define GTSAM_MAGIC_KEY

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/slam/smallExample.h>

using namespace std;
using namespace gtsam;
using namespace example;

static const Index _x_=0, _y_=1, _z_=2;

/* ************************************************************************* */
TEST( GaussianBayesNet, constructor )
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
  GaussianConditional x(_x_,d1,R11,_y_,S12, sigmas), y(_y_,d2,R22, sigmas);

  // check small example which uses constructor
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  CHECK( x.equals(*cbn[_x_]) );
  CHECK( y.equals(*cbn[_y_]) );
}

/* ************************************************************************* */
TEST( GaussianBayesNet, matrix )
{
  // Create a test graph
  GaussianBayesNet cbn = createSmallGaussianBayesNet();

  Matrix R; Vector d;
  boost::tie(R,d) = matrix(cbn); // find matrix and RHS

  Matrix R1 = Matrix_(2,2,
		      1.0, 1.0,
		      0.0, 1.0
    );
  Vector d1 = Vector_(2, 9.0, 5.0);

  EQUALITY(R,R1);
  CHECK(d==d1);
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimize )
{
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  VectorValues actual = optimize(cbn);

  VectorValues expected(vector<size_t>(2,1));
  expected[_x_] = Vector_(1,4.);
  expected[_y_] = Vector_(1,5.);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimize2 )
{

	// Create empty graph
	GaussianFactorGraph fg;
	SharedDiagonal noise = noiseModel::Unit::Create(1);

	fg.add(_y_, eye(1), 2*ones(1), noise);

	fg.add(_x_, eye(1),_y_, -eye(1), -ones(1), noise);

	fg.add(_y_, eye(1),_z_, -eye(1), -ones(1), noise);

	fg.add(_x_, -eye(1), _z_, eye(1), 2*ones(1), noise);

  VectorValues actual = *GaussianSequentialSolver(fg).optimize();

  VectorValues expected(vector<size_t>(3,1));
  expected[_x_] = Vector_(1,1.);
  expected[_y_] = Vector_(1,2.);
  expected[_z_] = Vector_(1,3.);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, backSubstitute )
{
	// y=R*x, x=inv(R)*y
	// 2 = 1 1  -1
	// 3     1   3
  GaussianBayesNet cbn = createSmallGaussianBayesNet();

  VectorValues y(vector<size_t>(2,1)), x(vector<size_t>(2,1));
  y[_x_] = Vector_(1,2.);
  y[_y_] = Vector_(1,3.);
  x[_x_] = Vector_(1,-1.);
  x[_y_] = Vector_(1, 3.);

  // test functional version
  VectorValues actual = backSubstitute(cbn,y);
  CHECK(assert_equal(x,actual));

  // test imperative version
  backSubstituteInPlace(cbn,y);
  CHECK(assert_equal(x,y));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, rhs )
{
	// y=R*x, x=inv(R)*y
	// 2 = 1 1  -1
	// 3     1   3
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
	VectorValues expected = gtsam::optimize(cbn);
	VectorValues d = rhs(cbn);
	VectorValues actual = backSubstitute(cbn, d);
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, rhs_with_sigmas )
{
	Matrix R11 = Matrix_(1, 1, 1.0), S12 = Matrix_(1, 1, 1.0);
	Matrix R22 = Matrix_(1, 1, 1.0);
	Vector d1(1), d2(1);
	d1(0) = 9;
	d2(0) = 5;
	Vector tau(1);
	tau(0) = 0.25;

	// define nodes and specify in reverse topological sort (i.e. parents last)
	GaussianConditional::shared_ptr Px_y(new GaussianConditional(_x_, d1, R11,
			_y_, S12, tau)), Py(new GaussianConditional(_y_, d2, R22, tau));
	GaussianBayesNet cbn;
	cbn.push_back(Px_y);
	cbn.push_back(Py);

	VectorValues expected = gtsam::optimize(cbn);
	VectorValues d = rhs(cbn);
	VectorValues actual = backSubstitute(cbn, d);
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, backSubstituteTranspose )
{
	// x=R'*y, y=inv(R')*x
	// 2 = 1    2
	// 5   1 1  3
  GaussianBayesNet cbn = createSmallGaussianBayesNet();

  VectorValues y(vector<size_t>(2,1)), x(vector<size_t>(2,1));
  x[_x_] = Vector_(1,2.);
  x[_y_] = Vector_(1,5.);
  y[_x_] = Vector_(1,2.);
  y[_y_] = Vector_(1,3.);

  // test functional version
  VectorValues actual = backSubstituteTranspose(cbn,x);
  CHECK(assert_equal(y,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
