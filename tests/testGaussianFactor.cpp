/**
 *  @file   testGaussianFactor.cpp
 *  @brief  Unit tests for Linear Factor
 *  @author Christian Potthast
 *  @author Frank Dellaert
 **/

#include <iostream>

#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp>
#include <boost/assign/std/map.hpp> // for insert
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/inference/inference-inl.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/nonlinear/Ordering.h>

using namespace std;
using namespace gtsam;
using namespace example;
using namespace boost;

static SharedDiagonal
	sigma0_1 = sharedSigma(2,0.1), sigma_02 = sharedSigma(2,0.2),
	constraintModel = noiseModel::Constrained::All(2);

/* ************************************************************************* */
TEST( GaussianFactor, linearFactor )
{
  Ordering ordering; ordering += "x1","x2","l1";

  Matrix I = eye(2);
	Vector b = Vector_(2, 2.0, -1.0);
	GaussianFactor expected(ordering["x1"], -10*I,ordering["x2"], 10*I, b, noiseModel::Unit::Create(2));

	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph(ordering);

	// get the factor "f2" from the factor graph
	GaussianFactor::shared_ptr lf = fg[1];

	// check if the two factors are the same
	CHECK(assert_equal(expected,*lf));
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, keys )
//{
//	// get the factor "f2" from the small linear factor graph
//  Ordering ordering; ordering += "x1","x2","l1";
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//	GaussianFactor::shared_ptr lf = fg[1];
//	list<Symbol> expected;
//	expected.push_back("x1");
//	expected.push_back("x2");
//	CHECK(lf->keys() == expected);
//}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, dimensions )
//{
//  // get the factor "f2" from the small linear factor graph
//  Ordering ordering; ordering += "x1","x2","l1";
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//  // Check a single factor
//  Dimensions expected;
//  insert(expected)("x1", 2)("x2", 2);
//  Dimensions actual = fg[1]->dimensions();
//  CHECK(expected==actual);
//}

/* ************************************************************************* */
TEST( GaussianFactor, getDim )
{
	// get a factor
  Ordering ordering; ordering += "x1","x2","l1";
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
	GaussianFactor::shared_ptr factor = fg[0];

	// get the size of a variable
	size_t actual = factor->getDim(factor->find(ordering["x1"]));

	// verify
	size_t expected = 2;
	CHECK(actual == expected);
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, combine )
//{
//	// create a small linear factor graph
//  Ordering ordering; ordering += "x1","x2","l1";
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
//	meas.push_back(make_pair("x2", Ax2));
//	meas.push_back(make_pair("l1", Al1));
//	meas.push_back(make_pair("x1", Ax1));
//	GaussianFactor expected(meas, b2, noiseModel::Diagonal::Sigmas(ones(4)));
//	CHECK(assert_equal(expected,combined));
//}

/* ************************************************************************* */
TEST( GaussianFactor, error )
{
	// create a small linear factor graph
  Ordering ordering; ordering += "x1","x2","l1";
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);

	// get the first factor from the factor graph
	GaussianFactor::shared_ptr lf = fg[0];

	// check the error of the first factor with noisy config
	VectorValues cfg = createZeroDelta(ordering);

	// calculate the error from the factor "f1"
	// note the error is the same as in testNonlinearFactor
	double actual = lf->error(cfg);
	DOUBLES_EQUAL( 1.0, actual, 0.00000001 );
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, eliminate )
//{
//	// create a small linear factor graph
//  Ordering ordering; ordering += "x1","x2","l1";
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
//	boost::tie(actualCG,actualLF) = combined.eliminate("x2");
//
//	// create expected Conditional Gaussian
//	Matrix I = eye(2)*sqrt(125.0);
//	Matrix R11 = I, S12 = -0.2*I, S13 = -0.8*I;
//	Vector d = I*Vector_(2,0.2,-0.14);
//
//	// Check the conditional Gaussian
//	GaussianConditional
//	expectedCG("x2", d, R11, "l1", S12, "x1", S13, repeat(2, 1.0));
//
//	// the expected linear factor
//	I = eye(2)/0.2236;
//	Matrix Bl1 = I, Bx1 = -I;
//	Vector b1 = I*Vector_(2,0.0,0.2);
//
//	GaussianFactor expectedLF("l1", Bl1, "x1", Bx1, b1, repeat(2,1.0));
//
//	// check if the result matches
//	CHECK(assert_equal(expectedCG,*actualCG,1e-3));
//	CHECK(assert_equal(expectedLF,*actualLF,1e-3));
//}

/* ************************************************************************* */
TEST( GaussianFactor, matrix )
{
	// create a small linear factor graph
  Ordering ordering; ordering += "x1","x2","l1";
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);

	// get the factor "f2" from the factor graph
	//GaussianFactor::shared_ptr lf = fg[1]; // NOTE: using the older version
	Vector b2 = Vector_(2, 0.2, -0.1);
	Matrix I = eye(2);
  // render with a given ordering
  Ordering ord;
  ord += "x1","x2";
	GaussianFactor::shared_ptr lf(new GaussianFactor(ord["x1"], -I, ord["x2"], I, b2, sigma0_1));

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
	shared_ptr<noiseModel::Gaussian> model = lf->get_model();
	model->WhitenSystem(A_act2, b_act2);
	EQUALITY(A_act1, A_act2);
	EQUALITY(b_act1, b_act2);
}

/* ************************************************************************* */
TEST( GaussianFactor, matrix_aug )
{
	// create a small linear factor graph
  Ordering ordering; ordering += "x1","x2","l1";
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);

	// get the factor "f2" from the factor graph
	//GaussianFactor::shared_ptr lf = fg[1];
	Vector b2 = Vector_(2, 0.2, -0.1);
	Matrix I = eye(2);
  // render with a given ordering
  Ordering ord;
  ord += "x1","x2";
	GaussianFactor::shared_ptr lf(new GaussianFactor(ord["x1"], -I, ord["x2"], I, b2, sigma0_1));


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
	shared_ptr<noiseModel::Gaussian> model = lf->get_model();
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
//  Ordering ordering; ordering += "x1","x2","l1";
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//	// get the factor "f2" from the factor graph
//	GaussianFactor::shared_ptr lf = fg[1];
//
//	// render with a given ordering
//	Ordering ord;
//	ord += "x1","x2";
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
//	CHECK(i==i1);
//	CHECK(j==j1);
//	CHECK(s==s1);
//}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactor, sparse2 )
//{
//	// create a small linear factor graph
//  Ordering ordering; ordering += "x1","x2","l1";
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//	// get the factor "f2" from the factor graph
//	GaussianFactor::shared_ptr lf = fg[1];
//
//	// render with a given ordering
//	Ordering ord;
//	ord += "x2","l1","x1";
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
//	CHECK(i==i1);
//	CHECK(j==j1);
//	CHECK(s==s1);
//}

/* ************************************************************************* */
TEST( GaussianFactor, size )
{
	// create a linear factor graph
  Ordering ordering; ordering += "x1","x2","l1";
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);

	// get some factors from the graph
	boost::shared_ptr<GaussianFactor> factor1 = fg[0];
	boost::shared_ptr<GaussianFactor> factor2 = fg[1];
	boost::shared_ptr<GaussianFactor> factor3 = fg[2];

	CHECK(factor1->size() == 1);
	CHECK(factor2->size() == 2);
	CHECK(factor3->size() == 2);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
