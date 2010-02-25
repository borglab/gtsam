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

#define GTSAM_MAGIC_KEY

#include "Matrix.h"
#include "Ordering.h"
#include "GaussianConditional.h"
#include "smallExample.h"

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
	Matrix I = eye(2);
	Vector b = Vector_(2, 2.0, -1.0);
	GaussianFactor expected("x1", -10*I,"x2", 10*I, b, noiseModel::Unit::Create(2));

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
	GaussianFactor lf("x1", -I, "x2", I, b, sigma0_1);

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

	// test transposeMultiplyAdd
	VectorConfig x;
	x.insert("x1",Vector_(2, 1.,2.));
	x.insert("x2",Vector_(2, 3.,4.));
	VectorConfig expectedX2 = x + 0.1 * (lf^e);
	lf.transposeMultiplyAdd(0.1,e,x);
	CHECK(assert_equal(expectedX2,x));
}

/* ************************************************************************* */
TEST( GaussianFactor, keys )
{
	// get the factor "f2" from the small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();
	GaussianFactor::shared_ptr lf = fg[1];
	list<Symbol> expected;
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
			-5., 0.,
			+0., -5.,
			10., 0.,
			+0., 10.);

	Matrix Al1 = Matrix_(4, 2,	// l1
			5., 0.,
			0., 5.,
			0., 0.,
			0., 0.);

	Matrix Ax1 = Matrix_(4, 2,	// x1
			0.00, 0., // f4
			0.00, 0., // f4
			-10., 0., // f2
			0.00, -10. // f2
	);

	// the RHS
	Vector b2(4);
	b2(0) = -1.0;
	b2(1) =  1.5;
	b2(2) =  2.0;
	b2(3) = -1.0;

	// use general constructor for making arbitrary factors
	vector<pair<Symbol, Matrix> > meas;
	meas.push_back(make_pair("x2", Ax2));
	meas.push_back(make_pair("l1", Al1));
	meas.push_back(make_pair("x1", Ax1));
	GaussianFactor expected(meas, b2, noiseModel::Diagonal::Sigmas(ones(4)));
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
	GaussianFactor::shared_ptr f1(new GaussianFactor("x1", A11, b*sigma1, sharedSigma(2,sigma1)));

	double sigma2 = 0.5;
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -1;
	b(0) = 4 ; b(1) = -5;
	GaussianFactor::shared_ptr f2(new GaussianFactor("x1", A11, b*sigma2, sharedSigma(2,sigma2)));

	double sigma3 = 0.25;
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -1;
	b(0) = 3 ; b(1) = -88;
	GaussianFactor::shared_ptr f3(new GaussianFactor("x1", A11, b*sigma3, sharedSigma(2,sigma3)));

	// TODO: find a real sigma value for this example
	double sigma4 = 0.1;
	A11(0,0) = 6; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = 7;
	b(0) = 5 ; b(1) = -6;
	GaussianFactor::shared_ptr f4(new GaussianFactor("x1", A11*sigma4, b*sigma4, sharedSigma(2,sigma4)));

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

	vector<pair<Symbol, Matrix> > meas;
	meas.push_back(make_pair("x1", A22));
	GaussianFactor expected(meas, exb, sigmas);
	CHECK(assert_equal(expected,combined));
}

/* ************************************************************************* */
TEST( GaussianFactor, linearFactorN){
	Matrix I = eye(2);
  vector<GaussianFactor::shared_ptr> f;
  SharedDiagonal model = sharedSigma(2,1.0);
  f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x1", I, Vector_(2,
			10.0, 5.0), model)));
	f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x1", -10 * I,
			"x2", 10 * I, Vector_(2, 1.0, -2.0), model)));
	f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x2", -10 * I,
			"x3", 10 * I, Vector_(2, 1.5, -1.5), model)));
	f.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x3", -10 * I,
			"x4", 10 * I, Vector_(2, 2.0, -1.0), model)));

  GaussianFactor combinedFactor(f);

  vector<pair<Symbol, Matrix> > combinedMeasurement;
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

  Vector sigmas = repeat(8,1.0);
  GaussianFactor expected(combinedMeasurement, b, sigmas);
  CHECK(assert_equal(expected,combinedFactor));
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
	Matrix I = eye(2)*sqrt(125.0);
	Matrix R11 = I, S12 = -0.2*I, S13 = -0.8*I;
	Vector d = I*Vector_(2,0.2,-0.14);

	// Check the conditional Gaussian
	GaussianConditional
	expectedCG("x2", d, R11, "l1", S12, "x1", S13, repeat(2, 1.0));

	// the expected linear factor
	I = eye(2)/0.2236;
	Matrix Bl1 = I, Bx1 = -I;
	Vector b1 = I*Vector_(2,0.0,0.2);

	GaussianFactor expectedLF("l1", Bl1, "x1", Bx1, b1, repeat(2,1.0));

	// check if the result matches
	CHECK(assert_equal(expectedCG,*actualCG,1e-3));
	CHECK(assert_equal(expectedLF,*actualLF,1e-3));
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

	vector<pair<Symbol, Matrix> > meas;
	meas.push_back(make_pair("x2", Ax2));
	meas.push_back(make_pair("l11", Al1x1));
	GaussianFactor combined(meas, b2, sigmas);

	// eliminate the combined factor
	GaussianConditional::shared_ptr actualCG;
	GaussianFactor::shared_ptr actualLF;
	boost::tie(actualCG,actualLF) = combined.eliminate("x2");

	// create expected Conditional Gaussian
	double oldSigma = 0.0894427; // from when R was made unit
	Matrix R11 = Matrix_(2,2,
			1.00,  0.00,
			0.00,  1.00
	)/oldSigma;
	Matrix S12 = Matrix_(2,4,
			-0.20, 0.00,-0.80, 0.00,
			+0.00,-0.20,+0.00,-0.80
	)/oldSigma;
	Vector d = Vector_(2,0.2,-0.14)/oldSigma;
	GaussianConditional expectedCG("x2",d,R11,"l11",S12,ones(2));
	CHECK(assert_equal(expectedCG,*actualCG,1e-4));

	// the expected linear factor
	double sigma = 0.2236;
	Matrix Bl1x1 = Matrix_(2,4,
			// l1          x1
			1.00, 0.00, -1.00,  0.00,
			0.00, 1.00, +0.00, -1.00
	)/sigma;
	Vector b1 =Vector_(2,0.0,0.894427);
	GaussianFactor expectedLF("l11", Bl1x1, b1, repeat(2,1.0));
	CHECK(assert_equal(expectedLF,*actualLF,1e-3));
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
	//GaussianFactor::shared_ptr lf = fg[1]; // NOTE: using the older version
	Vector b2 = Vector_(2, 0.2, -0.1);
	Matrix I = eye(2);
	GaussianFactor::shared_ptr lf(new GaussianFactor("x1", -I, "x2", I, b2, sigma0_1));

	// render with a given ordering
	Ordering ord;
	ord += "x1","x2";

	// Test whitened version
	Matrix A_act1; Vector b_act1;
	boost::tie(A_act1,b_act1) = lf->matrix(ord, true);

	Matrix A1 = Matrix_(2,4,
			-10.0,  0.0, 10.0,  0.0,
			000.0,-10.0,  0.0, 10.0 );
	Vector b1 = Vector_(2, 2.0, -1.0);

	EQUALITY(A_act1,A1);
	EQUALITY(b_act1,b1);

	// Test unwhitened version
	Matrix A_act2; Vector b_act2;
	boost::tie(A_act2,b_act2) = lf->matrix(ord, false);


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
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// get the factor "f2" from the factor graph
	//GaussianFactor::shared_ptr lf = fg[1];
	Vector b2 = Vector_(2, 0.2, -0.1);
	Matrix I = eye(2);
	GaussianFactor::shared_ptr lf(new GaussianFactor("x1", -I, "x2", I, b2, sigma0_1));

	// render with a given ordering
	Ordering ord;
	ord += "x1","x2";

	// Test unwhitened version
	Matrix Ab_act1;
	Ab_act1 = lf->matrix_augmented(ord, false);

	Matrix Ab1 = Matrix_(2,5,
			-1.0,  0.0, 1.0,  0.0,  0.2,
			00.0,- 1.0, 0.0,  1.0, -0.1 );

	EQUALITY(Ab_act1,Ab1);

	// Test whitened version
	Matrix Ab_act2;
	Ab_act2 = lf->matrix_augmented(ord, true);

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
TEST( GaussianFactor, tally_separator )
{
	GaussianFactor f("x1", eye(2), "x2", eye(2), "l1", eye(2), ones(2), sigma0_1);

	std::set<Symbol> act1, act2, act3;
	f.tally_separator("x1",	act1);
	f.tally_separator("x2",	act2);
	f.tally_separator("l1",	act3);

	CHECK(act1.size() == 2);
	CHECK(act1.count("x2") == 1);
	CHECK(act1.count("l1") == 1);

	CHECK(act2.size() == 2);
	CHECK(act2.count("x1") == 1);
	CHECK(act2.count("l1") == 1);

	CHECK(act3.size() == 2);
	CHECK(act3.count("x1") == 1);
	CHECK(act3.count("x2") == 1);
}

/* ************************************************************************* */
TEST( GaussianFactor, CONSTRUCTOR_GaussianConditional )
{
	Matrix R11 = eye(2);
	Matrix S12 = Matrix_(2,2,
			-0.200001, 0.00,
			+0.00,-0.200001
	);
	Vector d(2); d(0) = 2.23607; d(1) = -1.56525;
	Vector sigmas =repeat(2,0.29907);
	GaussianConditional::shared_ptr CG(new GaussianConditional("x2",d,R11,"l11",S12,sigmas));

	// Call the constructor we are testing !
	GaussianFactor actualLF(CG);

	GaussianFactor expectedLF("x2",R11,"l11",S12,d, sigmas);
	CHECK(assert_equal(expectedLF,actualLF,1e-5));
}

/* ************************************************************************* */
TEST ( GaussianFactor, constraint_eliminate1 )
{
	// construct a linear constraint
	Vector v(2); v(0)=1.2; v(1)=3.4;
	string key = "x0";
	GaussianFactor lc(key, eye(2), v, constraintModel);

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

	GaussianFactor lc("x", A1, "y", A2, b, constraintModel);

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
/* ************************************************************************* */
TEST ( GaussianFactor, combine_matrix ) {
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();
	Dimensions dimensions = fg.dimensions();

	// get two factors from it and insert the factors into a vector
	vector<GaussianFactor::shared_ptr> lfg;
	lfg.push_back(fg[4 - 1]);
	lfg.push_back(fg[2 - 1]);

	// combine in a factor
	Matrix Ab; SharedDiagonal noise;
	Ordering order; order += "x2", "l1", "x1";
	boost::tie(Ab, noise) = GaussianFactor::combineFactorsAndCreateMatrix(lfg, order, dimensions);

	// the expected augmented matrix
	Matrix expAb = Matrix_(4, 7,
			-5.,  0., 5., 0.,  0.,  0.,-1.0,
			+0., -5., 0., 5.,  0.,  0., 1.5,
			10.,  0., 0., 0.,-10.,  0., 2.0,
			+0., 10., 0., 0.,  0.,-10.,-1.0);

	// expected noise model
	SharedDiagonal expModel = noiseModel::Unit::Create(4);

	CHECK(assert_equal(expAb, Ab));
	CHECK(assert_equal(*expModel, *noise));
}

/* ************************************************************************* */
TEST ( GaussianFactor, exploding_MAST_factor ) {
	// Test derived from a crashing error in MAST
	// Works properly with the newer elimination code
	// This is only a test of execution without crashing

	Symbol lA2('l', 18295873486192642);
	Matrix A1 = eye(2);
	Vector b1 = zero(2);
	SharedDiagonal model1 = noiseModel::Isotropic::Sigma(2, 1.0/sqrt(2.0));
	GaussianFactor::shared_ptr f1(new GaussianFactor(lA2, A1, b1, model1));

	Matrix A2 = Matrix_(3,3,
			5.45735,	  1.94835,	 -1.68176,
				  0,	  10.2778,	 0.973046,
				  0,	        0,	   12.253);
	Vector b2 = Vector_(3, 1.29627e-16, 5.14706e-16, 4.19527e-16);
	SharedDiagonal model2 = noiseModel::Diagonal::Sigmas(ones(3));
	GaussianFactor::shared_ptr f2(new GaussianFactor(lA2, A2, b2, model2));

	GaussianFactorGraph fg;
	fg.push_back(f1);
	fg.push_back(f2);

	// works when using the newer implementation of eliminate
	GaussianConditional::shared_ptr cg = fg.eliminateOne(lA2, false);
	CHECK(true);
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
