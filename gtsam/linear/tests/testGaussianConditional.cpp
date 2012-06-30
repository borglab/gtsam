/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianConditional.cpp
 *  @brief  Unit tests for Conditional gaussian
 *  @author Christian Potthast
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianBayesNet.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/vector.hpp>

using namespace gtsam;
using namespace std;
using namespace boost::assign;

static const double tol = 1e-5;
static const Index _x_=0, _x1_=1, _l1_=2;

Matrix R = Matrix_(2,2,
		-12.1244,  -5.1962,
					0.,   4.6904);

/* ************************************************************************* */
TEST(GaussianConditional, constructor)
{
  Matrix S1 = Matrix_(2,2,
      -5.2786,  -8.6603,
      5.0254,   5.5432);
  Matrix S2 = Matrix_(2,2,
      -10.5573,  -5.9385,
      5.5737,   3.0153);
  Matrix S3 = Matrix_(2,2,
      -11.3820,  -7.2581,
      -3.0153,  -3.5635);

  Vector d = Vector_(2, 1.0, 2.0);
  Vector s = Vector_(2, 3.0, 4.0);

  list<pair<Index, Matrix> > terms;
  terms +=
      make_pair(3, S1),
      make_pair(5, S2),
      make_pair(7, S3);

  GaussianConditional actual(1, d, R, terms, s);

  GaussianConditional::const_iterator it = actual.beginFrontals();
  EXPECT(assert_equal(Index(1), *it));
  EXPECT(assert_equal(R, actual.get_R()));
  ++ it;
  EXPECT(it == actual.endFrontals());

  it = actual.beginParents();
  EXPECT(assert_equal(Index(3), *it));
  EXPECT(assert_equal(S1, actual.get_S(it)));

  ++ it;
  EXPECT(assert_equal(Index(5), *it));
  EXPECT(assert_equal(S2, actual.get_S(it)));

  ++ it;
  EXPECT(assert_equal(Index(7), *it));
  EXPECT(assert_equal(S3, actual.get_S(it)));

  ++it;
  EXPECT(it == actual.endParents());

  EXPECT(assert_equal(d, actual.get_d()));
  EXPECT(assert_equal(s, actual.get_sigmas()));

  // test copy constructor
  GaussianConditional copied(actual);
  EXPECT(assert_equal(d, copied.get_d()));
  EXPECT(assert_equal(s, copied.get_sigmas()));
  EXPECT(assert_equal(R, copied.get_R()));
}

/* ************************************************************************* */

GaussianConditional construct() {
	Vector d = Vector_(2, 1.0, 2.0);
	Vector s = Vector_(2, 3.0, 4.0);
	GaussianConditional::shared_ptr shared(new GaussianConditional(1, d, R, s));
	return *shared;
}

TEST(GaussianConditional, return_value)
{
	Vector d = Vector_(2, 1.0, 2.0);
	Vector s = Vector_(2, 3.0, 4.0);
  GaussianConditional copied = construct();
  EXPECT(assert_equal(d, copied.get_d()));
  EXPECT(assert_equal(s, copied.get_sigmas()));
  EXPECT(assert_equal(R, copied.get_R()));
}

/* ************************************************************************* */
TEST( GaussianConditional, equals )
{
  // create a conditional gaussian node
  Matrix A1(2,2);
  A1(0,0) = 1 ; A1(1,0) = 2;
  A1(0,1) = 3 ; A1(1,1) = 4;

  Matrix A2(2,2);
  A2(0,0) = 6 ; A2(1,0) = 0.2;
  A2(0,1) = 8 ; A2(1,1) = 0.4;

  Matrix R(2,2);
  R(0,0) = 0.1 ; R(1,0) = 0.3;
  R(0,1) = 0.0 ; R(1,1) = 0.34;

  Vector tau(2);
  tau(0) = 1.0;
  tau(1) = 0.34;

  Vector d(2);
  d(0) = 0.2; d(1) = 0.5;

  GaussianConditional
    expected(_x_,d, R, _x1_, A1, _l1_, A2, tau),
    actual(_x_,d, R, _x1_, A1, _l1_, A2, tau);

  EXPECT( expected.equals(actual) );
}

/* ************************************************************************* */
TEST( GaussianConditional, solve )
{
  //expected solution
  Vector expectedX(2);
  expectedX(0) = 20-3-11 ; expectedX(1) = 40-7-15;

  // create a conditional Gaussian node
  Matrix R = Matrix_(2,2,   1., 0.,
                            0., 1.);

  Matrix A1 = Matrix_(2,2,  1., 2.,
                            3., 4.);

  Matrix A2 = Matrix_(2,2,  5., 6.,
                            7., 8.);

  Vector d(2);
  d(0) = 20.0; d(1) = 40.0;

  Vector tau = ones(2);

  GaussianConditional cg(_x_, d, R, _x1_, A1, _l1_, A2, tau);

  Vector sx1(2);
  sx1(0) = 1.0; sx1(1) = 1.0;

  Vector sl1(2);
  sl1(0) = 1.0; sl1(1) = 1.0;

  VectorValues solution(vector<size_t>(3, 2));
  solution[_x_]  = d;
  solution[_x1_] = sx1; // parents
  solution[_l1_] = sl1;

  VectorValues expected(vector<size_t>(3, 2));
  expected[_x_] = expectedX;
  expected[_x1_] = sx1;
  expected[_l1_] = sl1;
  cg.solveInPlace(solution);

  EXPECT(assert_equal(expected, solution, 0.0001));
}

/* ************************************************************************* */
TEST( GaussianConditional, solve_simple )
{
	Matrix full_matrix = Matrix_(4, 7,
			1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.1,
			0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.2,
			0.0, 0.0, 3.0, 0.0, 4.0, 0.0, 0.3,
			0.0, 0.0, 0.0, 3.0, 0.0, 4.0, 0.4);

	// solve system as a non-multifrontal version first
	// 2 variables, frontal has dim=4
	vector<size_t> dims; dims += 4, 2, 1;
	GaussianConditional::rsd_type matrices(full_matrix, dims.begin(), dims.end());
	Vector sigmas = ones(4);
	vector<size_t> cgdims; cgdims += _x_, _x1_;
	GaussianConditional cg(cgdims.begin(), cgdims.end(), 1, matrices, sigmas);

	// partial solution
	Vector sx1 = Vector_(2, 9.0, 10.0);

	// elimination order; _x_, _x1_
	vector<size_t> vdim; vdim += 4, 2;
	VectorValues actual(vdim);
	actual[_x1_] = sx1; // parent

	VectorValues expected(vdim);
	expected[_x1_] = sx1;
	expected[_x_] = Vector_(4, -3.1,-3.4,-11.9,-13.2);

	// verify indices/size
	EXPECT_LONGS_EQUAL(2, cg.size());
	EXPECT_LONGS_EQUAL(4, cg.dim());

	// solve and verify
	cg.solveInPlace(actual);
	EXPECT(assert_equal(expected, actual, tol));
}

/* ************************************************************************* */
TEST( GaussianConditional, solve_multifrontal )
{
	// create full system, 3 variables, 2 frontals, all 2 dim
	Matrix full_matrix = Matrix_(4, 7,
			1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.1,
			0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.2,
			0.0, 0.0, 3.0, 0.0, 4.0, 0.0, 0.3,
			0.0, 0.0, 0.0, 3.0, 0.0, 4.0, 0.4);

	// 3 variables, all dim=2
	vector<size_t> dims; dims += 2, 2, 2, 1;
	GaussianConditional::rsd_type matrices(full_matrix, dims.begin(), dims.end());
	Vector sigmas = ones(4);
	vector<size_t> cgdims; cgdims += _x_, _x1_, _l1_;
	GaussianConditional cg(cgdims.begin(), cgdims.end(), 2, matrices, sigmas);

	EXPECT(assert_equal(Vector_(4, 0.1, 0.2, 0.3, 0.4), cg.get_d()));

	// partial solution
	Vector sl1 = Vector_(2, 9.0, 10.0);

	// elimination order; _x_, _x1_, _l1_
	VectorValues actual(vector<size_t>(3, 2));
	actual[_l1_] = sl1; // parent

	VectorValues expected(vector<size_t>(3, 2));
	expected[_x_] = Vector_(2, -3.1,-3.4);
	expected[_x1_] = Vector_(2, -11.9,-13.2);
	expected[_l1_] = sl1;

	// verify indices/size
	EXPECT_LONGS_EQUAL(3, cg.size());
	EXPECT_LONGS_EQUAL(4, cg.dim());

	// solve and verify
	cg.solveInPlace(actual);
	EXPECT(assert_equal(expected, actual, tol));

}

/* ************************************************************************* */
TEST( GaussianConditional, solveTranspose ) {
	static const Index _y_=1;
	/** create small Chordal Bayes Net x <- y
	 * x y d
	 * 1 1 9
	 *   1 5
	 */
	Matrix R11 = Matrix_(1, 1, 1.0), S12 = Matrix_(1, 1, 1.0);
	Matrix R22 = Matrix_(1, 1, 1.0);
	Vector d1(1), d2(1);
	d1(0) = 9;
	d2(0) = 5;
	Vector tau(1);
	tau(0) = 1.0;

	// define nodes and specify in reverse topological sort (i.e. parents last)
	GaussianConditional::shared_ptr Px_y(new GaussianConditional(_x_, d1, R11, _y_, S12, tau));
	GaussianConditional::shared_ptr Py(new GaussianConditional(_y_, d2, R22, tau));
	GaussianBayesNet cbn;
	cbn.push_back(Px_y);
	cbn.push_back(Py);

	// x=R'*y, y=inv(R')*x
	// 2 = 1    2
	// 5   1 1  3

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
TEST( GaussianConditional, computeInformation ) {

  // Create R matrix
  Matrix R = (Matrix(4,4) <<
      1, 2, 3, 4,
      0, 5, 6, 7,
      0, 0, 8, 9,
      0, 0, 0, 10).finished();

  // Create conditional
  GaussianConditional conditional(0, Vector::Zero(4), R, Vector::Constant(4, 1.0));

  // Expected information matrix (using permuted R)
  Matrix IExpected = R.transpose() * R;

  // Actual information matrix (conditional should permute R)
  Matrix IActual = conditional.computeInformation();
  EXPECT(assert_equal(IExpected, IActual));
}

/* ************************************************************************* */
TEST( GaussianConditional, isGaussianFactor ) {

  // Create R matrix
  Matrix R = (Matrix(4,4) <<
      1, 2, 3, 4,
      0, 5, 6, 7,
      0, 0, 8, 9,
      0, 0, 0, 10).finished();

  // Create a conditional
  GaussianConditional conditional(0, Vector::Zero(4), R, Vector::Constant(4, 1.0));

  // Expected information matrix computed by conditional
  Matrix IExpected = conditional.computeInformation();

  // Expected information matrix computed by a factor
  JacobianFactor jf = *conditional.toFactor();
  Matrix IActual = jf.getA(jf.begin()).transpose() * jf.getA(jf.begin());

  EXPECT(assert_equal(IExpected, IActual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
