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

#include <tests/smallExample.h>
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
	sigma0_1 = noiseModel::Isotropic::Sigma(2,0.1), sigma_02 = noiseModel::Isotropic::Sigma(2,0.2),
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
	GaussianFactorGraph fg = example::createGaussianFactorGraph(ordering);

	// get the factor kf2 from the factor graph
	JacobianFactor::shared_ptr lf =
	    boost::dynamic_pointer_cast<JacobianFactor>(fg[1]);

	// check if the two factors are the same
	EXPECT(assert_equal(expected,*lf));
}

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

/* ************************************************************************* */
TEST( GaussianFactor, matrix )
{
	const Key kx1 = X(1), kx2 = X(2), kl1 = L(1);
	// create a small linear factor graph
  Ordering ordering; ordering += kx1,kx2,kl1;
  GaussianFactorGraph fg = example::createGaussianFactorGraph(ordering);

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
  GaussianFactorGraph fg = example::createGaussianFactorGraph(ordering);

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
