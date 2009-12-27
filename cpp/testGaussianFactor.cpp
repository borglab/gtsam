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

#include <CppUnitLite/TestHarness.h>

#include "Matrix.h"
#include "Ordering.h"
#include "GaussianConditional.h"
#include "smallExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( GaussianFactor, linearFactor )
{
	Matrix I = eye(2);
	Vector b = Vector_(2,0.2,-0.1);
	double sigma = 0.1;
	GaussianFactor expected("x1", -I, "x2", I, b, sigma);

	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get the factor "f2" from the factor graph
	GaussianFactor::shared_ptr lf = fg[1];

	// check if the two factors are the same
	CHECK(assert_equal(expected,*lf));
}

/* ************************************************************************* */
TEST( GaussianFactor, operators )
{
	Matrix I = eye(2);
	Vector b = Vector_(2,0.2,-0.1);
	double sigma = 0.1;
	GaussianFactor lf("x1", -I, "x2", I, b, sigma);

	VectorConfig c;
	c.insert("x1",Vector_(2,10.,20.));
	c.insert("x2",Vector_(2,30.,60.));

	// test A*x
	Vector expectedE = Vector_(2,200.,400.), e = lf*c;
	CHECK(assert_equal(expectedE,e));

	// test A^e
	VectorConfig expectedX;
	expectedX.insert("x1",Vector_(2,-2000.,-4000.));
	expectedX.insert("x2",Vector_(2, 2000., 4000.));
	CHECK(assert_equal(expectedX,lf^e));
}

/* ************************************************************************* */
TEST( GaussianFactor, keys )
{
	// get the factor "f2" from the small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();
	GaussianFactor::shared_ptr lf = fg[1];
	list<string> expected;
	expected.push_back("x1");
	expected.push_back("x2");
	CHECK(lf->keys() == expected);
}

/* ************************************************************************* */
TEST( GaussianFactor, dimensions )
{
  // get the factor "f2" from the small linear factor graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // Check a single factor
  Dimensions expected;
  insert(expected)("x1", 2)("x2", 2);
  Dimensions actual = fg[1]->dimensions();
  CHECK(expected==actual);
}

/* ************************************************************************* */
TEST( GaussianFactor, getDim )
{
	// get a factor
	GaussianFactorGraph fg = createGaussianFactorGraph();
	GaussianFactor::shared_ptr factor = fg[0];

	// get the size of a variable
	size_t actual = factor->getDim("x1");

	// verify
	size_t expected = 2;
	CHECK(actual == expected);
}

/* ************************************************************************* */
TEST( GaussianFactor, combine )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get two factors from it and insert the factors into a vector
	vector<GaussianFactor::shared_ptr> lfg;
	lfg.push_back(fg[4 - 1]);
	lfg.push_back(fg[2 - 1]);

	// combine in a factor
	GaussianFactor combined(lfg);

	// sigmas
	double sigma2 = 0.1;
	double sigma4 = 0.2;
	Vector sigmas = Vector_(4, sigma4, sigma4, sigma2, sigma2);

	// the expected combined linear factor
	Matrix Ax2 = Matrix_(4, 2, // x2
			-1., 0.,
			+0., -1.,
			1., 0.,
			+0., 1.);

	Matrix Al1 = Matrix_(4, 2,	// l1
			1., 0.,
			0., 1.,
			0., 0.,
			0., 0.);

	Matrix Ax1 = Matrix_(4, 2,	// x1
			0.00, 0., // f4
			0.00, 0., // f4
			-1., 0., // f2
			0.00, -1. // f2
	);

	// the RHS
	Vector b2(4);
	b2(0) = -0.2;
	b2(1) = 0.3;
	b2(2) = 0.2;
	b2(3) = -0.1;

	// use general constructor for making arbitrary factors
	vector<pair<string, Matrix> > meas;
	meas.push_back(make_pair("x2", Ax2));
	meas.push_back(make_pair("l1", Al1));
	meas.push_back(make_pair("x1", Ax1));
	GaussianFactor expected(meas, b2, sigmas);
	CHECK(assert_equal(expected,combined));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, combine2){
	double sigma1 = 0.0957;
	Matrix A11(2,2);
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0;       A11(1,1) = 1;
	Vector b(2);
	b(0) = 2; b(1) = -1;
	GaussianFactor::shared_ptr f1(new GaussianFactor("x1", A11, b*sigma1, sigma1));

	double sigma2 = 0.5;
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -1;
	b(0) = 4 ; b(1) = -5;
	GaussianFactor::shared_ptr f2(new GaussianFactor("x1", A11, b*sigma2, sigma2));

	double sigma3 = 0.25;
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -1;
	b(0) = 3 ; b(1) = -88;
	GaussianFactor::shared_ptr f3(new GaussianFactor("x1", A11, b*sigma3, sigma3));

	// TODO: find a real sigma value for this example
	double sigma4 = 0.1;
	A11(0,0) = 6; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = 7;
	b(0) = 5 ; b(1) = -6;
	GaussianFactor::shared_ptr f4(new GaussianFactor("x1", A11*sigma4, b*sigma4, sigma4));

	vector<GaussianFactor::shared_ptr> lfg;
	lfg.push_back(f1);
	lfg.push_back(f2);
	lfg.push_back(f3);
	lfg.push_back(f4);
	GaussianFactor combined(lfg);

	Vector sigmas = Vector_(8, sigma1, sigma1, sigma2, sigma2, sigma3, sigma3, sigma4, sigma4);
	Matrix A22(8,2);
	A22(0,0) = 1;   A22(0,1) =  0;
	A22(1,0) = 0;   A22(1,1) = 1;
	A22(2,0) = 1;   A22(2,1) =  0;
	A22(3,0) = 0;   A22(3,1) = -1;
	A22(4,0) = 1;   A22(4,1) =  0;
	A22(5,0) = 0;   A22(5,1) = -1;
	A22(6,0) = 0.6; A22(6,1) =  0;
	A22(7,0) = 0;   A22(7,1) =  0.7;
	Vector exb(8);
	exb(0) = 2*sigma1 ; exb(1) = -1*sigma1;  exb(2) = 4*sigma2 ; exb(3) = -5*sigma2;
	exb(4) = 3*sigma3 ; exb(5) = -88*sigma3; exb(6) = 5*sigma4 ; exb(7) = -6*sigma4;

	vector<pair<string, Matrix> > meas;
	meas.push_back(make_pair("x1", A22));
	GaussianFactor expected(meas, exb, sigmas);
	CHECK(assert_equal(expected,combined));
}

/* ************************************************************************* */
TEST( GaussianFactor, linearFactorN){
  vector<GaussianFactor::shared_ptr> f;
  f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x1", Matrix_(2,2,
      1.0, 0.0,
      0.0, 1.0),
      Vector_(2,
      10.0, 5.0), 1)));
  f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x1", Matrix_(2,2,
      -10.0, 0.0,
      0.0, -10.0),
      "x2", Matrix_(2,2,
      10.0, 0.0,
      0.0, 10.0),
      Vector_(2,
      1.0, -2.0), 1)));
  f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x2", Matrix_(2,2,
      -10.0, 0.0,
      0.0, -10.0),
      "x3", Matrix_(2,2,
      10.0, 0.0,
      0.0, 10.0),
      Vector_(2,
      1.5, -1.5), 1)));
  f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x3", Matrix_(2,2,
      -10.0, 0.0,
      0.0, -10.0),
      "x4", Matrix_(2,2,
      10.0, 0.0,
      0.0, 10.0),
      Vector_(2,
      2.0, -1.0), 1)));

  GaussianFactor combinedFactor(f);

  vector<pair<string, Matrix> > combinedMeasurement;
  combinedMeasurement.push_back(make_pair("x1", Matrix_(8,2,
      1.0,  0.0,
      0.0,  1.0,
    -10.0,  0.0,
      0.0,-10.0,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0)));
  combinedMeasurement.push_back(make_pair("x2", Matrix_(8,2,
      0.0,  0.0,
      0.0,  0.0,
     10.0,  0.0,
      0.0, 10.0,
    -10.0,  0.0,
      0.0,-10.0,
      0.0,  0.0,
      0.0,  0.0)));
  combinedMeasurement.push_back(make_pair("x3", Matrix_(8,2,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0,
     10.0,  0.0,
      0.0, 10.0,
    -10.0,  0.0,
      0.0,-10.0)));
  combinedMeasurement.push_back(make_pair("x4", Matrix_(8,2,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
     10.0, 0.0,
      0.0,10.0)));
  Vector b = Vector_(8,
      10.0, 5.0, 1.0, -2.0, 1.5, -1.5, 2.0, -1.0);

  GaussianFactor expected(combinedMeasurement, b, 1.);
  CHECK(combinedFactor.equals(expected));
}

/* ************************************************************************* */
TEST( GaussianFactor, error )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get the first factor from the factor graph
	GaussianFactor::shared_ptr lf = fg[0];

	// check the error of the first factor with noisy config
	VectorConfig cfg = createZeroDelta();

	// calculate the error from the factor "f1"
	// note the error is the same as in testNonlinearFactor
	double actual = lf->error(cfg);
	DOUBLES_EQUAL( 1.0, actual, 0.00000001 );
}

/* ************************************************************************* */
TEST( GaussianFactor, eliminate )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get two factors from it and insert the factors into a vector
	vector<GaussianFactor::shared_ptr> lfg;
	lfg.push_back(fg[4 - 1]);
	lfg.push_back(fg[2 - 1]);

	// combine in a factor
	GaussianFactor combined(lfg);

	// eliminate the combined factor
	GaussianConditional::shared_ptr actualCG;
	GaussianFactor::shared_ptr actualLF;
	boost::tie(actualCG,actualLF) = combined.eliminate("x2");

	// create expected Conditional Gaussian
	Matrix R11 = Matrix_(2,2,
			1.0, 0.0,
			0.0, 1.0
	);
	Matrix S12 = Matrix_(2,2,
			-0.2, 0.0,
			+0.0,-0.2
	);
	Matrix S13 = Matrix_(2,2,
			-0.8, 0.0,
			+0.0,-0.8
	);
	Vector d(2); d(0) = 0.2; d(1) = -0.14;

	Vector sigmas(2);
	sigmas(0) = 1/sqrt(125.0);
	sigmas(1) = 1/sqrt(125.0);

	// Check the conditional Gaussian
	GaussianConditional expectedCG("x2", d,R11,"l1",S12,"x1",S13,sigmas);

	// the expected linear factor
	double sigma = 0.2236;
	Matrix Bl1 = Matrix_(2,2,
			// l1
			1.00, 0.00,
			0.00, 1.00
	);

	Matrix Bx1 = Matrix_(2,2,
			// x1
			-1.00,  0.00,
			+0.00, -1.00
	);

	// the RHS
	Vector b1(2); b1(0) = 0.0; b1(1) = 0.2;

	GaussianFactor expectedLF("l1", Bl1, "x1", Bx1, b1, sigma);

	// check if the result matches
	CHECK(assert_equal(expectedCG,*actualCG,1e-4));
	CHECK(assert_equal(expectedLF,*actualLF,1e-5));
}


/* ************************************************************************* */
TEST( GaussianFactor, eliminate2 )
{
	// sigmas
	double sigma1 = 0.2;
	double sigma2 = 0.1;
	Vector sigmas = Vector_(4, sigma1, sigma1, sigma2, sigma2);

	// the combined linear factor
	Matrix Ax2 = Matrix_(4,2,
			// x2
			-1., 0.,
			+0.,-1.,
			1., 0.,
			+0.,1.
	);

	Matrix Al1x1 = Matrix_(4,4,
			// l1   x1
			1., 0., 0.00,  0., // f4
			0., 1., 0.00,  0., // f4
			0., 0., -1.,  0., // f2
			0., 0., 0.00,-1.  // f2
	);

	// the RHS
	Vector b2(4);
	b2(0) = -0.2;
	b2(1) =  0.3;
	b2(2) =  0.2;
	b2(3) = -0.1;

	vector<pair<string, Matrix> > meas;
	meas.push_back(make_pair("x2", Ax2));
	meas.push_back(make_pair("l1x1", Al1x1));
	GaussianFactor combined(meas, b2, sigmas);

	// eliminate the combined factor
	GaussianConditional::shared_ptr actualCG;
	GaussianFactor::shared_ptr actualLF;
	boost::tie(actualCG,actualLF) = combined.eliminate("x2");

	// create expected Conditional Gaussian
	Matrix R11 = Matrix_(2,2,
			1.00,  0.00,
			0.00,  1.00
	);
	Matrix S12 = Matrix_(2,4,
			-0.20, 0.00,-0.80, 0.00,
			+0.00,-0.20,+0.00,-0.80
	);
	Vector d(2); d(0) = 0.2; d(1) = -0.14;

	Vector x2Sigmas(2);
	x2Sigmas(0) = 0.0894427;
	x2Sigmas(1) = 0.0894427;

	GaussianConditional expectedCG("x2",d,R11,"l1x1",S12,x2Sigmas);

	// the expected linear factor
	double sigma = 0.2236;
	Matrix Bl1x1 = Matrix_(2,4,
			// l1          x1
			1.00, 0.00, -1.00,  0.00,
			0.00, 1.00, +0.00, -1.00
	);

	// the RHS
	Vector b1(2); b1(0) = 0.0; b1(1) = 0.894427;

	GaussianFactor expectedLF("l1x1", Bl1x1, b1*sigma, sigma);

	// check if the result matches
	CHECK(assert_equal(expectedCG,*actualCG,1e-4));
	CHECK(assert_equal(expectedLF,*actualLF,1e-5));
}

/* ************************************************************************* */
TEST( GaussianFactor, default_error )
{
	GaussianFactor f;
	VectorConfig c;
	double actual = f.error(c);
	CHECK(actual==0.0);
}

//* ************************************************************************* */
TEST( GaussianFactor, eliminate_empty )
{
	// create an empty factor
	GaussianFactor f;

	// eliminate the empty factor
	GaussianConditional::shared_ptr actualCG;
	GaussianFactor::shared_ptr actualLF;
	boost::tie(actualCG,actualLF) = f.eliminate("x2");

	// expected Conditional Gaussian is just a parent-less node with P(x)=1
	GaussianConditional expectedCG("x2");

	// expected remaining factor is still empty :-)
	GaussianFactor expectedLF;

	// check if the result matches
	CHECK(actualCG->equals(expectedCG));
	CHECK(actualLF->equals(expectedLF));
}

//* ************************************************************************* */
TEST( GaussianFactor, empty )
{
	// create an empty factor
	GaussianFactor f;
	CHECK(f.empty()==true);
}

/* ************************************************************************* */
TEST( GaussianFactor, matrix )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get the factor "f2" from the factor graph
	GaussianFactor::shared_ptr lf = fg[1];

	// render with a given ordering
	Ordering ord;
	ord += "x1","x2";

	Matrix A; Vector b;
	boost::tie(A,b) = lf->matrix(ord);

	Matrix A1 = Matrix_(2,4,
			-10.0,  0.0, 10.0,  0.0,
			000.0,-10.0,  0.0, 10.0 );
	Vector b1 = Vector_(2, 2.0, -1.0);

	EQUALITY(A,A1);
	EQUALITY(b,b1);
}

/* ************************************************************************* */
TEST( GaussianFactor, matrix_aug )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get the factor "f2" from the factor graph
	GaussianFactor::shared_ptr lf = fg[1];

	// render with a given ordering
	Ordering ord;
	ord += "x1","x2";

	Matrix Ab;
	Ab = lf->matrix_augmented(ord);

	Matrix Ab1 = Matrix_(2,5,
			-1.0,  0.0, 1.0,  0.0,  0.2,
			00.0,- 1.0, 0.0,  1.0, -0.1 );

	EQUALITY(Ab,Ab1);
}

/* ************************************************************************* */
// small aux. function to print out lists of anything
template<class T>
void print(const list<T>& i) {
	copy(i.begin(), i.end(), ostream_iterator<T> (cout, ","));
	cout << endl;
}

/* ************************************************************************* */
TEST( GaussianFactor, sparse )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get the factor "f2" from the factor graph
	GaussianFactor::shared_ptr lf = fg[1];

	// render with a given ordering
	Ordering ord;
	ord += "x1","x2";

	list<int> i,j;
	list<double> s;
	boost::tie(i,j,s) = lf->sparse(fg.columnIndices(ord));

	list<int> i1,j1;
	i1 += 1,2,1,2;
	j1 += 1,2,3,4;

	list<double> s1;
	s1 += -10,-10,10,10;

	CHECK(i==i1);
	CHECK(j==j1);
	CHECK(s==s1);
}

/* ************************************************************************* */
TEST( GaussianFactor, sparse2 )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get the factor "f2" from the factor graph
	GaussianFactor::shared_ptr lf = fg[1];

	// render with a given ordering
	Ordering ord;
	ord += "x2","l1","x1";

	list<int> i,j;
	list<double> s;
	boost::tie(i,j,s) = lf->sparse(fg.columnIndices(ord));

	list<int> i1,j1;
	i1 += 1,2,1,2;
	j1 += 5,6,1,2;

	list<double> s1;
	s1 += -10,-10,10,10;

	CHECK(i==i1);
	CHECK(j==j1);
	CHECK(s==s1);
}

/* ************************************************************************* */
TEST( GaussianFactor, size )
{
	// create a linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get some factors from the graph
	boost::shared_ptr<GaussianFactor> factor1 = fg[0];
	boost::shared_ptr<GaussianFactor> factor2 = fg[1];
	boost::shared_ptr<GaussianFactor> factor3 = fg[2];

	CHECK(factor1->size() == 1);
	CHECK(factor2->size() == 2);
	CHECK(factor3->size() == 2);
}

/* ************************************************************************* */
TEST( GaussianFactor, CONSTRUCTOR_GaussianConditional )
{
	Matrix R11 = Matrix_(2,2,
			1.00,  0.00,
			0.00,  1.00
	);
	Matrix S12 = Matrix_(2,2,
			-0.200001, 0.00,
			+0.00,-0.200001
	);
	Vector d(2); d(0) = 2.23607; d(1) = -1.56525;

	Vector sigmas(2);
	sigmas(0) = 0.29907;
	sigmas(1) = 0.29907;

	GaussianConditional::shared_ptr CG(new GaussianConditional("x2",d,R11,"l1x1",S12,sigmas));
	GaussianFactor actualLF(CG);
	//  actualLF.print();
	GaussianFactor expectedLF("x2",R11,"l1x1",S12,d, sigmas(0));

	CHECK(assert_equal(expectedLF,actualLF,1e-5));
}

/* ************************************************************************* */
TEST( GaussianFactor, alphaFactor )
{
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get alphafactor for first factor in fg at zero, in gradient direction
  VectorConfig x = createZeroDelta();
	VectorConfig d = fg.gradient(x);
	GaussianFactor::shared_ptr factor = fg[0];
	GaussianFactor::shared_ptr actual = factor->alphaFactor(x,d);

	// calculate expected
	Matrix A = Matrix_(2,1,30.0,5.0);
	Vector b = Vector_(2,-0.1,-0.1);
	Vector sigmas = Vector_(2,0.1,0.1);
	GaussianFactor expected("alpha",A,b,sigmas);

	CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST ( GaussianFactor, constraint_eliminate1 )
{
	// construct a linear constraint
	Vector v(2); v(0)=1.2; v(1)=3.4;
	string key = "x0";
	GaussianFactor lc(key, eye(2), v, 0.0);

	// eliminate it
	GaussianConditional::shared_ptr actualCG;
	GaussianFactor::shared_ptr actualLF;
	boost::tie(actualCG,actualLF) = lc.eliminate("x0");

	// verify linear factor
	CHECK(actualLF->size() == 0);

	// verify conditional Gaussian
	Vector sigmas = Vector_(2, 0.0, 0.0);
	GaussianConditional expCG("x0", v, eye(2), sigmas);
	CHECK(assert_equal(expCG, *actualCG));
}

/* ************************************************************************* */
TEST ( GaussianFactor, constraint_eliminate2 )
{
	// Construct a linear constraint
	// RHS
	Vector b(2); b(0)=3.0; b(1)=4.0;

	// A1 - invertible
	Matrix A1(2,2);
	A1(0,0) = 1.0 ; A1(0,1) = 2.0;
	A1(1,0) = 2.0 ; A1(1,1) = 1.0;

	// A2 - not invertible
	Matrix A2(2,2);
	A2(0,0) = 1.0 ; A2(0,1) = 2.0;
	A2(1,0) = 2.0 ; A2(1,1) = 4.0;

	GaussianFactor lc("x", A1, "y", A2, b, 0.0);

	// eliminate x and verify results
	GaussianConditional::shared_ptr actualCG;
	GaussianFactor::shared_ptr actualLF;
	boost::tie(actualCG, actualLF) = lc.eliminate("x");

	// LF should be null
	GaussianFactor expectedLF;
	CHECK(assert_equal(*actualLF, expectedLF));

	// verify CG
	Matrix R = Matrix_(2, 2,
			1.0,    2.0,
			0.0,    1.0);
	Matrix S = Matrix_(2,2,
			1.0,    2.0,
			0.0,    0.0);
	Vector d = Vector_(2, 3.0, 0.6666);
	GaussianConditional expectedCG("x", d, R, "y", S, zero(2));
	CHECK(assert_equal(expectedCG, *actualCG, 1e-4));
}

/* ************************************************************************* *
TEST ( GaussianFactor, constraint_eliminate3 )
{
	// This test shows that ordering matters if there are non-invertible
	// blocks, as this example can be eliminated if x is first, but not
	// if y is first.

	// Construct a linear constraint
	// RHS
	Vector b(2); b(0)=3.0; b(1)=4.0;

	// A1 - invertible
	Matrix A1(2,2);
	A1(0,0) = 1.0 ; A1(0,1) = 2.0;
	A1(1,0) = 2.0 ; A1(1,1) = 1.0;

	// A2 - not invertible
	Matrix A2(2,2);
	A2(0,0) = 1.0 ; A2(0,1) = 2.0;
	A2(1,0) = 2.0 ; A2(1,1) = 4.0;

	GaussianFactor lc("x", A1, "y", A2, b, 0.0);

	// eliminate y from original graph
	// NOTE: this will throw an exception, as
	// the leading matrix is rank deficient
	GaussianConditional::shared_ptr actualCG;
	GaussianFactor::shared_ptr actualLF;
	try {
		boost::tie(actualCG, actualLF) = lc.eliminate("y");
		CHECK(false);
	} catch (domain_error) {
		CHECK(true);
	}
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
