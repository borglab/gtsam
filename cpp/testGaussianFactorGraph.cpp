/**
 *  @file   testGaussianFactorGraph.cpp
 *  @brief  Unit tests for Linear Factor Graph
 *  @author Christian Potthast
 **/

#include <string.h>
#include <iostream>
using namespace std;

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Matrix.h"
#include "Ordering.h"
#include "smallExample.h"
#include "GaussianBayesNet.h"
#include "inference-inl.h" // needed for eliminate and marginals

using namespace gtsam;

double tol=1e-4;

/* ************************************************************************* */
/* unit test for equals (GaussianFactorGraph1 == GaussianFactorGraph2)           */
/* ************************************************************************* */
TEST( GaussianFactorGraph, equals ){

  GaussianFactorGraph fg = createGaussianFactorGraph();
  GaussianFactorGraph fg2 = createGaussianFactorGraph();
  CHECK(fg.equals(fg2));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, error )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  VectorConfig cfg = createZeroDelta();

  // note the error is the same as in testNonlinearFactorGraph as a
  // zero delta config in the linear graph is equivalent to noisy in
  // non-linear, which is really linear under the hood
  double actual = fg.error(cfg);
  DOUBLES_EQUAL( 5.625, actual, 1e-9 );
}

/* ************************************************************************* */
/* unit test for find seperator                                              */
/* ************************************************************************* */
TEST( GaussianFactorGraph, find_separator )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();

  set<string> separator = fg.find_separator("x2");
  set<string> expected;
  expected.insert("x1");
  expected.insert("l1");

  CHECK(separator.size()==expected.size());
  set<string>::iterator it1 = separator.begin(), it2 = expected.begin();
  for(; it1!=separator.end(); it1++, it2++)
    CHECK(*it1 == *it2);
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, combine_factors_x1 )
{
  // create a small example for a linear factor graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // create sigmas
  double sigma1 = 0.1;
  double sigma2 = 0.1;
  double sigma3 = 0.2;
  Vector sigmas = Vector_(6, sigma1, sigma1, sigma2, sigma2, sigma3, sigma3);

  // combine all factors
  GaussianFactor::shared_ptr actual = removeAndCombineFactors(fg,"x1");

  // the expected linear factor
  Matrix Al1 = Matrix_(6,2,
			 0., 0.,
			 0., 0.,
			 0., 0.,
			 0., 0.,
			 1., 0.,
			 0., 1.
			 );

  Matrix Ax1 = Matrix_(6,2,
			 1.,   0.,
			 0.00, 1.,
			 -1.,  0.,
			 0.00,-1.,
			 -1.,   0.,
			 00.,  -1.
			 );

  Matrix Ax2 = Matrix_(6,2,
			 0., 0.,
			 0., 0.,
			 1., 0.,
			 +0.,1.,
			 0., 0.,
			 0., 0.
			 );

  // the expected RHS vector
  Vector b(6);
  b(0) = -1*sigma1;
  b(1) = -1*sigma1;
  b(2) =  2*sigma2;
  b(3) = -1*sigma2;
  b(4) =  0*sigma3;
  b(5) =  1*sigma3;

  vector<pair<string, Matrix> > meas;
  meas.push_back(make_pair("l1", Al1));
  meas.push_back(make_pair("x1", Ax1));
  meas.push_back(make_pair("x2", Ax2));
  GaussianFactor expected(meas, b, sigmas);
  //GaussianFactor expected("l1", Al1, "x1", Ax1, "x2", Ax2, b);

  // check if the two factors are the same
  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, combine_factors_x2 )
{
 // create a small example for a linear factor graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // determine sigmas
  double sigma1 = 0.1;
  double sigma2 = 0.2;
  Vector sigmas = Vector_(4, sigma1, sigma1, sigma2, sigma2);

  // combine all factors
  GaussianFactor::shared_ptr actual = removeAndCombineFactors(fg,"x2");

  // the expected linear factor
  Matrix Al1 = Matrix_(4,2,
			 // l1
			 0., 0.,
			 0., 0.,
			 1., 0.,
			 0., 1.
			 );

  Matrix Ax1 = Matrix_(4,2,
                         // x1
			 -1.,  0.,  // f2
			 0.00,-1.,  // f2
			 0.00,  0., // f4
			 0.00,  0.  // f4
			 );

  Matrix Ax2 = Matrix_(4,2,
			 // x2
			 1., 0.,
			 +0.,1.,
			 -1., 0.,
			 +0.,-1.
			 );

  // the expected RHS vector
  Vector b(4);
  b(0) = 2*sigma1;
  b(1) = -1*sigma1;
  b(2) = -1*sigma2;
  b(3) = 1.5*sigma2;

  vector<pair<string, Matrix> > meas;
  meas.push_back(make_pair("l1", Al1));
  meas.push_back(make_pair("x1", Ax1));
  meas.push_back(make_pair("x2", Ax2));
  GaussianFactor expected(meas, b, sigmas);

  // check if the two factors are the same
  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */

TEST( GaussianFactorGraph, eliminateOne_x1 )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  GaussianConditional::shared_ptr actual = fg.eliminateOne("x1");

  // create expected Conditional Gaussian
  Matrix I = eye(2), R11 = I, S12 = -0.111111*I, S13 = -0.444444*I;
  Vector d = Vector_(2, -0.133333, -0.0222222), sigma = repeat(2, 1./15);
  GaussianConditional expected("x1",d,R11,"l1",S12,"x2",S13,sigma);

  CHECK(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */

TEST( GaussianFactorGraph, eliminateOne_x2 )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  GaussianConditional::shared_ptr actual = fg.eliminateOne("x2");

  // create expected Conditional Gaussian
  Matrix I = eye(2), R11 = I, S12 = -0.2*I, S13 = -0.8*I;
  Vector d = Vector_(2, 0.2, -0.14), sigma = repeat(2, 0.0894427);
  GaussianConditional expected("x2",d,R11,"l1",S12,"x1",S13,sigma);

  CHECK(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateOne_l1 )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  GaussianConditional::shared_ptr actual = fg.eliminateOne("l1");

  // create expected Conditional Gaussian
  Matrix I = eye(2), R11 = I, S12 = -0.5*I, S13 = -0.5*I;
  Vector d = Vector_(2, -0.1, 0.25), sigma = repeat(2, 0.141421);
  GaussianConditional expected("l1",d,R11,"x1",S12,"x2",S13,sigma);

  CHECK(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateAll )
{
  // create expected Chordal bayes Net
  Matrix I = eye(2);

  Vector d1 = Vector_(2, -0.1,-0.1);
  GaussianBayesNet expected = simpleGaussian("x1",d1,0.1);

  Vector d2 = Vector_(2, 0.0, 0.2), sigma2 = repeat(2,0.149071);
  push_front(expected,"l1",d2, I,"x1", (-1)*I,sigma2);

  Vector d3 = Vector_(2, 0.2, -0.14), sigma3 = repeat(2,0.0894427);
  push_front(expected,"x2",d3, I,"l1", (-0.2)*I, "x1", (-0.8)*I, sigma3);

  // Check one ordering
  GaussianFactorGraph fg1 = createGaussianFactorGraph();
  Ordering ordering;
  ordering += "x2","l1","x1";
  GaussianBayesNet actual = fg1.eliminate(ordering);
  CHECK(assert_equal(expected,actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, add_priors )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  GaussianFactorGraph actual = fg.add_priors(3);
  GaussianFactorGraph expected = createGaussianFactorGraph();
  Matrix A = eye(2);
  Vector b = zero(2);
  double sigma = 3.0;
  expected.push_back(GaussianFactor::shared_ptr(new GaussianFactor("l1",A,b,sigma)));
  expected.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x1",A,b,sigma)));
  expected.push_back(GaussianFactor::shared_ptr(new GaussianFactor("x2",A,b,sigma)));
  CHECK(assert_equal(expected,actual)); // Fails
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, copying )
{
  // Create a graph
  GaussianFactorGraph actual = createGaussianFactorGraph();

  // Copy the graph !
  GaussianFactorGraph copy = actual;

  // now eliminate the copy
  Ordering ord1;
  ord1 += "x2","l1","x1";
  GaussianBayesNet actual1 = copy.eliminate(ord1);

  // Create the same graph, but not by copying
  GaussianFactorGraph expected = createGaussianFactorGraph();

  // and check that original is still the same graph
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, matrix )
{
  // Create a graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // render with a given ordering
  Ordering ord;
  ord += "x2","l1","x1";

  Matrix A; Vector b;
  boost::tie(A,b) = fg.matrix(ord);

  Matrix A1 = Matrix_(2*4,3*2,
		     +0.,  0.,  0.,  0., 10.,  0., // unary factor on x1 (prior)
		     +0.,  0.,  0.,  0.,  0., 10.,
		     10.,  0.,  0.,  0.,-10.,  0., // binary factor on x2,x1 (odometry)
		     +0., 10.,  0.,  0.,  0.,-10.,
		     +0.,  0.,  5.,  0., -5.,  0., // binary factor on l1,x1 (z1)
		     +0.,  0.,  0.,  5.,  0., -5.,
		     -5.,  0.,  5.,  0.,  0.,  0., // binary factor on x2,l1 (z2)
		     +0., -5.,  0.,  5.,  0.,  0.
    );
  Vector b1 = Vector_(8,-1., -1., 2., -1., 0., 1., -1., 1.5);

  EQUALITY(A,A1);
  CHECK(b==b1);
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, sparse )
{
	// create a small linear factor graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// render with a given ordering
	Ordering ord;
  ord += "x2","l1","x1";

	Matrix ijs = fg.sparse(ord);

	EQUALITY(ijs, Matrix_(3, 14,
		// f(x1)   f(x2,x1)            f(l1,x1)         f(x2,l1)
		+1., 2.,   3., 4.,  3.,  4.,   5.,6., 5., 6.,   7., 8.,7.,8.,
		+5., 6.,   1., 2.,  5.,  6.,   3.,4., 5., 6.,   1., 2.,3.,4.,
		10.,10.,  10.,10.,-10.,-10.,   5.,5.,-5.,-5.,  -5.,-5.,5.,5.));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, CONSTRUCTOR_GaussianBayesNet )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // render with a given ordering
  Ordering ord;
  ord += "x2","l1","x1";
  GaussianBayesNet CBN = fg.eliminate(ord);

  // True GaussianFactorGraph
  GaussianFactorGraph fg2(CBN);
  GaussianBayesNet CBN2 = fg2.eliminate(ord);
  CHECK(assert_equal(CBN,CBN2));

  // Base FactorGraph only
  FactorGraph<GaussianFactor> fg3(CBN);
  GaussianBayesNet CBN3 = gtsam::eliminate<GaussianFactor,GaussianConditional>(fg3,ord);
  CHECK(assert_equal(CBN,CBN3));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, GET_ORDERING)
{
  Ordering expected;
  expected += "l1","x1","x2";
  GaussianFactorGraph fg = createGaussianFactorGraph();
  Ordering actual = fg.getOrdering();
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, OPTIMIZE )
{
	// create a graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// create an ordering
	Ordering ord = fg.getOrdering();

	// optimize the graph
	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected = createCorrectDelta();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, COMBINE_GRAPHS_INPLACE)
{
	// create a test graph
	GaussianFactorGraph fg1 = createGaussianFactorGraph();

	// create another factor graph
	GaussianFactorGraph fg2 = createGaussianFactorGraph();

	// get sizes
	int size1 = fg1.size();
	int size2 = fg2.size();

	// combine them
	fg1.combine(fg2);

	CHECK(size1+size2 == fg1.size());
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, COMBINE_GRAPHS)
{
	// create a test graph
	GaussianFactorGraph fg1 = createGaussianFactorGraph();

	// create another factor graph
	GaussianFactorGraph fg2 = createGaussianFactorGraph();

	// get sizes
	int size1 = fg1.size();
	int size2 = fg2.size();

	// combine them
	GaussianFactorGraph fg3 = GaussianFactorGraph::combine2(fg1, fg2);

	CHECK(size1+size2 == fg3.size());
}

/* ************************************************************************* */
// print a vector of ints if needed for debugging
void print(vector<int> v) {
	for (int k = 0; k < v.size(); k++)
		cout << v[k] << " ";
	cout << endl;
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, factor_lookup)
{
	// create a test graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

	// ask for all factor indices connected to x1
	list<int> x1_factors = fg.factors("x1");
	int x1_indices[] = { 0, 1, 2 };
	list<int> x1_expected(x1_indices, x1_indices + 3);
	CHECK(x1_factors==x1_expected);

	// ask for all factor indices connected to x2
	list<int> x2_factors = fg.factors("x2");
	int x2_indices[] = { 1, 3 };
	list<int> x2_expected(x2_indices, x2_indices + 2);
	CHECK(x2_factors==x2_expected);
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, findAndRemoveFactors )
{
	// create the graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

  // We expect to remove these three factors: 0, 1, 2
  GaussianFactor::shared_ptr f0 = fg[0];
  GaussianFactor::shared_ptr f1 = fg[1];
  GaussianFactor::shared_ptr f2 = fg[2];

  // call the function
  vector<GaussianFactor::shared_ptr> factors = fg.findAndRemoveFactors("x1");

  // Check the factors
  CHECK(f0==factors[0]);
  CHECK(f1==factors[1]);
  CHECK(f2==factors[2]);

  // CHECK if the factors are deleted from the factor graph
  LONGS_EQUAL(1,fg.nrFactors());
  }

/* ************************************************************************* */
TEST( GaussianFactorGraph, findAndRemoveFactors_twice )
{
	// create the graph
	GaussianFactorGraph fg = createGaussianFactorGraph();

  // We expect to remove these three factors: 0, 1, 2
  GaussianFactor::shared_ptr f0 = fg[0];
  GaussianFactor::shared_ptr f1 = fg[1];
  GaussianFactor::shared_ptr f2 = fg[2];

  // call the function
  vector<GaussianFactor::shared_ptr> factors = fg.findAndRemoveFactors("x1");

  // Check the factors
  CHECK(f0==factors[0]);
  CHECK(f1==factors[1]);
  CHECK(f2==factors[2]);

  factors = fg.findAndRemoveFactors("x1");
  CHECK(factors.size() == 0);

  // CHECK if the factors are deleted from the factor graph
  LONGS_EQUAL(1,fg.nrFactors());
  }

/* ************************************************************************* */
TEST(GaussianFactorGraph, createSmoother)
{
	GaussianFactorGraph fg1 = createSmoother(2);
	LONGS_EQUAL(3,fg1.size());
	GaussianFactorGraph fg2 = createSmoother(3);
	LONGS_EQUAL(5,fg2.size());
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, variables )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  Dimensions expected;
  insert(expected)("l1", 2)("x1", 2)("x2", 2);
  Dimensions actual = fg.dimensions();
  CHECK(expected==actual);
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, keys )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  Ordering expected;
  expected += "l1","x1","x2";
  CHECK(assert_equal(expected,fg.keys()));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, involves )
{
  GaussianFactorGraph fg = createGaussianFactorGraph();
  CHECK(fg.involves("l1"));
  CHECK(fg.involves("x1"));
  CHECK(fg.involves("x2"));
  CHECK(!fg.involves("x3"));
}

/* ************************************************************************* */
// Tests ported from ConstrainedGaussianFactorGraph
/* ************************************************************************* */

/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_simple )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createSimpleConstraintGraph();

	// eliminate and solve
	Ordering ord;
	ord += "x", "y";
	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected = createSimpleConstraintConfig();
	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_single )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createSingleConstraintGraph();

	// eliminate and solve
	Ordering ord;
	ord += "x", "y";
	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected = createSingleConstraintConfig();
	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_single2 )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createSingleConstraintGraph();

	// eliminate and solve
	Ordering ord;
	ord += "y", "x";
	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected = createSingleConstraintConfig();
	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_multi1 )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createMultiConstraintGraph();

	// eliminate and solve
	Ordering ord;
	ord += "x", "y", "z";
	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected = createMultiConstraintConfig();
	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_multi2 )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createMultiConstraintGraph();

	// eliminate and solve
	Ordering ord;
	ord += "z", "x", "y";
	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected = createMultiConstraintConfig();
	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
