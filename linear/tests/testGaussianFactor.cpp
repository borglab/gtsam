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
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/inference/inference-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost;

static SharedDiagonal
	sigma0_1 = sharedSigma(2,0.1), sigma_02 = sharedSigma(2,0.2),
	constraintModel = noiseModel::Constrained::All(2);

/* ************************************************************************* */
TEST( GaussianFactor, constructor)
{
	Vector b = Vector_(3, 1., 2., 3.);
	SharedDiagonal noise = noiseModel::Diagonal::Sigmas(Vector_(3,1.,1.,1.));
	Symbol x0('x',0), x1('x',1);
	std::list<std::pair<Symbol, Matrix> > terms;
	terms.push_back(make_pair(x0, eye(2)));
	terms.push_back(make_pair(x1, 2.*eye(2)));
	GaussianFactor actual(terms, b, noise);
	GaussianFactor expected(x0, eye(2), x1, 2.*eye(2), b, noise);
	CHECK(assert_equal(expected, actual));
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
// small aux. function to print out lists of anything
template<class T>
void print(const list<T>& i) {
	copy(i.begin(), i.end(), ostream_iterator<T> (cout, ","));
	cout << endl;
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
TEST( GaussianFactor, CONSTRUCTOR_GaussianConditionalConstrained )
{
  Matrix Ax = eye(2);
  Vector b = Vector_(2, 3.0, 5.0);
  SharedDiagonal noisemodel = noiseModel::Constrained::All(2);
  GaussianFactor::shared_ptr expected(new GaussianFactor("x0", Ax, b, noisemodel));
  GaussianFactorGraph graph;
  graph.push_back(expected);

  GaussianConditional::shared_ptr conditional = graph.eliminateOne("x0");
  GaussianFactor actual(conditional);

  CHECK(assert_equal(*expected, actual));
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
TEST( GaussianFactor, erase)
{
	Vector b = Vector_(3, 1., 2., 3.);
	SharedDiagonal noise = noiseModel::Diagonal::Sigmas(Vector_(3,1.,1.,1.));
	Symbol x0('x',0), x1('x',1);
	std::list<std::pair<Symbol, Matrix> > terms;
	terms.push_back(make_pair(x0, eye(2)));
	terms.push_back(make_pair(x1, 2.*eye(2)));

	GaussianFactor actual(terms, b, noise);
	int erased = actual.erase_A(x0);

	LONGS_EQUAL(1, erased);
	GaussianFactor expected(x1, 2.*eye(2), b, noise);
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianFactor, eliminateMatrix)
{
	Matrix Ab = Matrix_(3, 4,
			1., 2., 0., 3.,
			0., 4., 5., 6.,
			0., 0., 7., 8.);
	SharedDiagonal model(Vector_(3, 0.5, 0.5, 0.5));
	Ordering frontals; frontals += "x1", "x2";
	Ordering separator; separator += "x3";
	Dimensions dimensions;
	dimensions.insert(make_pair("x1", 1));
	dimensions.insert(make_pair("x2", 1));
	dimensions.insert(make_pair("x3", 1));

	GaussianFactor::shared_ptr factor;
	GaussianBayesNet bn;
	boost::tie(bn, factor) =
			GaussianFactor::eliminateMatrix(Ab, model, frontals, separator, dimensions);

	GaussianBayesNet bn_expected;
	GaussianBayesNet::sharedConditional conditional1(new GaussianConditional("x1", Vector_(1, 6.), Matrix_(1, 1, 2.),
			"x2", Matrix_(1, 1, 4.), "x3", Matrix_(1, 1, 0.), Vector_(1, 1.)));
	GaussianBayesNet::sharedConditional conditional2(new GaussianConditional("x2", Vector_(1, 12.), Matrix_(1, 1, 8.),
			"x3", Matrix_(1, 1, 10.), Vector_(1, 1.)));
	bn_expected.push_back(conditional1);
	bn_expected.push_back(conditional2);
	CHECK(assert_equal(bn_expected, bn));

	GaussianFactor factor_expected("x3", Matrix_(1, 1, 14.), Vector_(1, 16.), SharedDiagonal(Vector_(1, 1.)));
	CHECK(assert_equal(factor_expected, *factor));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
