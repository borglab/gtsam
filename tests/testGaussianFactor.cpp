/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactor.cpp
 *  @brief  Unit tests for Linear Factor
 *  @author Christian Potthast
 *  @author Frank Dellaert
 **/

#include <gtsam/slam/smallExample.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp>
#include <boost/assign/std/map.hpp> // for insert
using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

static SharedDiagonal
	sigma0_1 = sharedSigma(2,0.1), sigma_02 = sharedSigma(2,0.2),
	constraintModel = noiseModel::Constrained::All(2);

//const Key kx1 = X(1), kx2 = X(2), kl1 = L(1); // FIXME: throws exception

/* ************************************************************************* */
TEST( GaussianFactor, linearFactor )
{
	const Key kx1 = X(1), kx2 = X(2), kl1 = L(1);
  Ordering ordering; ordering += kx1,kx2,kl1;

  Matrix I = eye(2);
	Vector b = Vector_(2, 2.0, -1.0);
	JacobianFactor expected(ordering[kx1], -10*I,ordering[kx2], 10*I, b, noiseModel::Unit::Create(2));

	// create a small linear factor graph
	FactorGraph<JacobianFactor> fg = example::createGaussianFactorGraph(ordering);

	// get the factor kf2 from the factor graph
	JacobianFactor::shared_ptr lf = fg[1];

	// check if the two factors are the same
	EXPECT(assert_equal(expected,*lf));
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, keys )
//{
//	// get the factor kf2 from the small linear factor graph
//  Ordering ordering; ordering += kx1,kx2,kl1;
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//	GaussianFactor::shared_ptr lf = fg[1];
//	list<Symbol> expected;
//	expected.push_back(kx1);
//	expected.push_back(kx2);
//	EXPECT(lf->keys() == expected);
//}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, dimensions )
//{
//  // get the factor kf2 from the small linear factor graph
//  Ordering ordering; ordering += kx1,kx2,kl1;
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//  // Check a single factor
//  Dimensions expected;
//  insert(expected)(kx1, 2)(kx2, 2);
//  Dimensions actual = fg[1]->dimensions();
//  EXPECT(expected==actual);
//}

/* ************************************************************************* */
TEST( GaussianFactor, getDim )
{
	const Key kx1 = X(1), kx2 = X(2), kl1 = L(1);
	// get a factor
  Ordering ordering; ordering += kx1,kx2,kl1;
  GaussianFactorGraph fg = example::createGaussianFactorGraph(ordering);
	GaussianFactor::shared_ptr factor = fg[0];

	// get the size of a variable
	size_t actual = factor->getDim(factor->find(ordering[kx1]));

	// verify
	size_t expected = 2;
	EXPECT_LONGS_EQUAL(expected, actual);
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, combine )
//{
//	// create a small linear factor graph
//  Ordering ordering; ordering += kx1,kx2,kl1;
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//	// get two factors from it and insert the factors into a vector
//	vector<GaussianFactor::shared_ptr> lfg;
//	lfg.push_back(fg[4 - 1]);
//	lfg.push_back(fg[2 - 1]);
//
//	// combine in a factor
//	GaussianFactor combined(lfg);
//
//	// sigmas
//	double sigma2 = 0.1;
//	double sigma4 = 0.2;
//	Vector sigmas = Vector_(4, sigma4, sigma4, sigma2, sigma2);
//
//	// the expected combined linear factor
//	Matrix Ax2 = Matrix_(4, 2, // x2
//			-5., 0.,
//			+0., -5.,
//			10., 0.,
//			+0., 10.);
//
//	Matrix Al1 = Matrix_(4, 2,	// l1
//			5., 0.,
//			0., 5.,
//			0., 0.,
//			0., 0.);
//
//	Matrix Ax1 = Matrix_(4, 2,	// x1
//			0.00, 0., // f4
//			0.00, 0., // f4
//			-10., 0., // f2
//			0.00, -10. // f2
//	);
//
//	// the RHS
//	Vector b2(4);
//	b2(0) = -1.0;
//	b2(1) =  1.5;
//	b2(2) =  2.0;
//	b2(3) = -1.0;
//
//	// use general constructor for making arbitrary factors
//	vector<pair<Symbol, Matrix> > meas;
//	meas.push_back(make_pair(kx2, Ax2));
//	meas.push_back(make_pair(kl1, Al1));
//	meas.push_back(make_pair(kx1, Ax1));
//	GaussianFactor expected(meas, b2, noiseModel::Diagonal::Sigmas(ones(4)));
//	EXPECT(assert_equal(expected,combined));
//}

/* ************************************************************************* */
TEST( GaussianFactor, error )
{
	const Key kx1 = X(1), kx2 = X(2), kl1 = L(1);
	// create a small linear factor graph
  Ordering ordering; ordering += kx1,kx2,kl1;
  GaussianFactorGraph fg = example::createGaussianFactorGraph(ordering);

	// get the first factor from the factor graph
	GaussianFactor::shared_ptr lf = fg[0];

	// check the error of the first factor with noisy config
	VectorValues cfg = example::createZeroDelta(ordering);

	// calculate the error from the factor kf1
	// note the error is the same as in testNonlinearFactor
	double actual = lf->error(cfg);
	DOUBLES_EQUAL( 1.0, actual, 0.00000001 );
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, eliminate )
//{
//	// create a small linear factor graph
//  Ordering ordering; ordering += kx1,kx2,kl1;
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//	// get two factors from it and insert the factors into a vector
//	vector<GaussianFactor::shared_ptr> lfg;
//	lfg.push_back(fg[4 - 1]);
//	lfg.push_back(fg[2 - 1]);
//
//	// combine in a factor
//	GaussianFactor combined(lfg);
//
//	// eliminate the combined factor
//	GaussianConditional::shared_ptr actualCG;
//	GaussianFactor::shared_ptr actualLF;
//	boost::tie(actualCG,actualLF) = combined.eliminate(kx2);
//
//	// create expected Conditional Gaussian
//	Matrix I = eye(2)*sqrt(125.0);
//	Matrix R11 = I, S12 = -0.2*I, S13 = -0.8*I;
//	Vector d = I*Vector_(2,0.2,-0.14);
//
//	// Check the conditional Gaussian
//	GaussianConditional
//	expectedCG(kx2, d, R11, kl1, S12, kx1, S13, repeat(2, 1.0));
//
//	// the expected linear factor
//	I = eye(2)/0.2236;
//	Matrix Bl1 = I, Bx1 = -I;
//	Vector b1 = I*Vector_(2,0.0,0.2);
//
//	GaussianFactor expectedLF(kl1, Bl1, kx1, Bx1, b1, repeat(2,1.0));
//
//	// check if the result matches
//	EXPECT(assert_equal(expectedCG,*actualCG,1e-3));
//	EXPECT(assert_equal(expectedLF,*actualLF,1e-3));
//}

/* ************************************************************************* */
TEST( GaussianFactor, matrix )
{
	const Key kx1 = X(1), kx2 = X(2), kl1 = L(1);
	// create a small linear factor graph
  Ordering ordering; ordering += kx1,kx2,kl1;
  FactorGraph<JacobianFactor> fg = example::createGaussianFactorGraph(ordering);

	// get the factor kf2 from the factor graph
	//GaussianFactor::shared_ptr lf = fg[1]; // NOTE: using the older version
	Vector b2 = Vector_(2, 0.2, -0.1);
	Matrix I = eye(2);
  // render with a given ordering
  Ordering ord;
  ord += kx1,kx2;
	JacobianFactor::shared_ptr lf(new JacobianFactor(ord[kx1], -I, ord[kx2], I, b2, sigma0_1));

	// Test whitened version
	Matrix A_act1; Vector b_act1;
	boost::tie(A_act1,b_act1) = lf->matrix(true);

	Matrix A1 = Matrix_(2,4,
			-10.0,  0.0, 10.0,  0.0,
			000.0,-10.0,  0.0, 10.0 );
	Vector b1 = Vector_(2, 2.0, -1.0);

	EQUALITY(A_act1,A1);
	EQUALITY(b_act1,b1);

	// Test unwhitened version
	Matrix A_act2; Vector b_act2;
	boost::tie(A_act2,b_act2) = lf->matrix(false);


	Matrix A2 = Matrix_(2,4,
			-1.0,  0.0, 1.0,  0.0,
			000.0,-1.0,  0.0, 1.0 );
	//Vector b2 = Vector_(2, 2.0, -1.0);

	EQUALITY(A_act2,A2);
	EQUALITY(b_act2,b2);

	// Ensure that whitening is consistent
	boost::shared_ptr<noiseModel::Gaussian> model = lf->get_model();
	model->WhitenSystem(A_act2, b_act2);
	EQUALITY(A_act1, A_act2);
	EQUALITY(b_act1, b_act2);
}

/* ************************************************************************* */
TEST( GaussianFactor, matrix_aug )
{
	const Key kx1 = X(1), kx2 = X(2), kl1 = L(1);
	// create a small linear factor graph
  Ordering ordering; ordering += kx1,kx2,kl1;
  FactorGraph<JacobianFactor> fg = example::createGaussianFactorGraph(ordering);

	// get the factor kf2 from the factor graph
	//GaussianFactor::shared_ptr lf = fg[1];
	Vector b2 = Vector_(2, 0.2, -0.1);
	Matrix I = eye(2);
  // render with a given ordering
  Ordering ord;
  ord += kx1,kx2;
	JacobianFactor::shared_ptr lf(new JacobianFactor(ord[kx1], -I, ord[kx2], I, b2, sigma0_1));


	// Test unwhitened version
	Matrix Ab_act1;
	Ab_act1 = lf->matrix_augmented(false);

	Matrix Ab1 = Matrix_(2,5,
			-1.0,  0.0, 1.0,  0.0,  0.2,
			00.0,- 1.0, 0.0,  1.0, -0.1 );

	EQUALITY(Ab_act1,Ab1);

	// Test whitened version
	Matrix Ab_act2;
	Ab_act2 = lf->matrix_augmented(true);

	Matrix Ab2 = Matrix_(2,5,
		   -10.0,  0.0, 10.0,  0.0,  2.0,
			00.0, -10.0,  0.0, 10.0, -1.0 );

	EQUALITY(Ab_act2,Ab2);

	// Ensure that whitening is consistent
	boost::shared_ptr<noiseModel::Gaussian> model = lf->get_model();
	model->WhitenInPlace(Ab_act1);
	EQUALITY(Ab_act1, Ab_act2);
}

/* ************************************************************************* */
// small aux. function to print out lists of anything
template<class T>
void print(const list<T>& i) {
	copy(i.begin(), i.end(), ostream_iterator<T> (cout, ","));
	cout << endl;
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, sparse )
//{
//	// create a small linear factor graph
//  Ordering ordering; ordering += kx1,kx2,kl1;
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//	// get the factor kf2 from the factor graph
//	GaussianFactor::shared_ptr lf = fg[1];
//
//	// render with a given ordering
//	Ordering ord;
//	ord += kx1,kx2;
//
//	list<int> i,j;
//	list<double> s;
//	boost::tie(i,j,s) = lf->sparse(fg.columnIndices(ord));
//
//	list<int> i1,j1;
//	i1 += 1,2,1,2;
//	j1 += 1,2,3,4;
//
//	list<double> s1;
//	s1 += -10,-10,10,10;
//
//	EXPECT(i==i1);
//	EXPECT(j==j1);
//	EXPECT(s==s1);
//}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, sparse2 )
//{
//	// create a small linear factor graph
//  Ordering ordering; ordering += kx1,kx2,kl1;
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//	// get the factor kf2 from the factor graph
//	GaussianFactor::shared_ptr lf = fg[1];
//
//	// render with a given ordering
//	Ordering ord;
//	ord += kx2,kl1,kx1;
//
//	list<int> i,j;
//	list<double> s;
//	boost::tie(i,j,s) = lf->sparse(fg.columnIndices(ord));
//
//	list<int> i1,j1;
//	i1 += 1,2,1,2;
//	j1 += 5,6,1,2;
//
//	list<double> s1;
//	s1 += -10,-10,10,10;
//
//	EXPECT(i==i1);
//	EXPECT(j==j1);
//	EXPECT(s==s1);
//}

/* ************************************************************************* */
TEST( GaussianFactor, size )
{
	// create a linear factor graph
	const Key kx1 = X(1), kx2 = X(2), kl1 = L(1);
  Ordering ordering; ordering += kx1,kx2,kl1;
  GaussianFactorGraph fg = example::createGaussianFactorGraph(ordering);

	// get some factors from the graph
	boost::shared_ptr<GaussianFactor> factor1 = fg[0];
	boost::shared_ptr<GaussianFactor> factor2 = fg[1];
	boost::shared_ptr<GaussianFactor> factor3 = fg[2];

	EXPECT_LONGS_EQUAL(1, factor1->size());
	EXPECT_LONGS_EQUAL(2, factor2->size());
	EXPECT_LONGS_EQUAL(2, factor3->size());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
