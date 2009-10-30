/**
 *  @file   testLinearFactorGraph.cpp
 *  @brief  Unit tests for Linear Factor Graph
 *  @author Christian Potthast
 **/

/*STL/C++*/
#include <iostream>
using namespace std;

#include <CppUnitLite/TestHarness.h>
#include <string.h>
#include <boost/tuple/tuple.hpp>
#include "Matrix.h"
#include "smallExample.h"

using namespace gtsam;

double tol=1e-4;

/* ************************************************************************* */
/* unit test for equals (LinearFactorGraph1 == LinearFactorGraph2)           */ 
/* ************************************************************************* */
TEST( LinearFactorGraph, equals ){

  LinearFactorGraph fg = createLinearFactorGraph();
  LinearFactorGraph fg2 = createLinearFactorGraph();
  CHECK(fg.equals(fg2));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, error )
{
  LinearFactorGraph fg = createLinearFactorGraph();
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
TEST( LinearFactorGraph, find_separator )
{	
  LinearFactorGraph fg = createLinearFactorGraph();

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
TEST( LinearFactorGraph, combine_factors_x1 )
{	
  // create a small example for a linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();

  // combine all factors
  LinearFactor::shared_ptr actual = fg.removeAndCombineFactors("x1");

  // the expected linear factor
  Matrix Al1 = Matrix_(6,2,
			 0., 0.,
			 0., 0.,
			 0., 0.,
			 0., 0.,
			 5., 0.,
			 0., 5.
			 );

  Matrix Ax1 = Matrix_(6,2,
			 10.,   0.,
			 0.00, 10.,
			 -10.,  0.,
			 0.00,-10.,
			 -5.,   0.,
			 00.,  -5.
			 );

  Matrix Ax2 = Matrix_(6,2,
			 0., 0.,
			 0., 0.,
			 10., 0.,
			 +0.,10.,
			 0., 0.,
			 0., 0.
			 );

  // the expected RHS vector
  Vector b(6);
  b(0) = -1;
  b(1) = -1;
  b(2) =  2;
  b(3) = -1;
  b(4) =  0;
  b(5) =  1;

  LinearFactor expected("l1", Al1, "x1", Ax1, "x2", Ax2, b);

  // check if the two factors are the same
  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, combine_factors_x2 )
{	
 // create a small example for a linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();

  // combine all factors
  LinearFactor::shared_ptr actual = fg.removeAndCombineFactors("x2");

  // the expected linear factor
  Matrix Al1 = Matrix_(4,2,
			 // l1
			 0., 0.,
			 0., 0.,
			 5., 0.,
			 0., 5.
			 );

  Matrix Ax1 = Matrix_(4,2,
                         // x1
			 -10.,  0.,  // f2
			 0.00,-10.,  // f2
			 0.00,  0., // f4
			 0.00,  0.  // f4
			 );

  Matrix Ax2 = Matrix_(4,2,
			 // x2
			 10., 0.,
			 +0.,10.,
			 -5., 0.,
			 +0.,-5.
			 );

  // the expected RHS vector
  Vector b(4);
  b(0) = 2;
  b(1) = -1;
  b(2) = -1;
  b(3) = 1.5;

  LinearFactor expected("l1", Al1, "x1", Ax1, "x2", Ax2, b);

  // check if the two factors are the same
  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */

TEST( LinearFactorGraph, eliminateOne_x1 )
{
  LinearFactorGraph fg = createLinearFactorGraph();
  ConditionalGaussian::shared_ptr actual =
  		fg.eliminateOne<ConditionalGaussian>("x1");

  // create expected Conditional Gaussian
  Matrix R11 = Matrix_(2,2,
			 15.0, 00.0,
			 00.0, 15.0
			 );
  Matrix S12 = Matrix_(2,2,
			 -1.66667, 0.00,
			 +0.00,-1.66667
			 );
  Matrix S13 = Matrix_(2,2,
			 -6.66667, 0.00,
			 +0.00,-6.66667
			 );
  Vector d(2); d(0) = -2; d(1) = -1.0/3.0;
  ConditionalGaussian expected(d,R11,"l1",S12,"x2",S13);

  CHECK(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
 
TEST( LinearFactorGraph, eliminateOne_x2 )
{
  LinearFactorGraph fg = createLinearFactorGraph();
  ConditionalGaussian::shared_ptr actual =
  		fg.eliminateOne<ConditionalGaussian>("x2");

  // create expected Conditional Gaussian
  Matrix R11 = Matrix_(2,2,
			 11.1803,  0.00,
			 0.00, 11.1803
			 );
  Matrix S12 = Matrix_(2,2,
			 -2.23607, 0.00,
			 +0.00,-2.23607
			 );
  Matrix S13 = Matrix_(2,2,
			 -8.94427, 0.00,
			 +0.00,-8.94427
			 );
  Vector d(2); d(0) = 2.23607; d(1) = -1.56525;
  ConditionalGaussian expected(d,R11,"l1",S12,"x1",S13);

  CHECK(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, eliminateOne_l1 )
{
  LinearFactorGraph fg = createLinearFactorGraph();
  ConditionalGaussian::shared_ptr actual =
  		fg.eliminateOne<ConditionalGaussian>("l1");

  // create expected Conditional Gaussian
  Matrix R11 = Matrix_(2,2,
			 7.07107, 0.00,
			 0.00, 7.07107
			 );
  Matrix S12 = Matrix_(2,2,
			 -3.53553, 0.00,
			 +0.00,-3.53553
			 );
  Matrix S13 = Matrix_(2,2,
			 -3.53553, 0.00,
			 +0.00,-3.53553
			 );
  Vector d(2); d(0) = -0.707107; d(1) = 1.76777;
  ConditionalGaussian expected(d,R11,"x1",S12,"x2",S13);

  CHECK(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, eliminateAll )
{
  // create expected Chordal bayes Net
  double data1[] = { 10, 0.0,
                    0.0, 10};
  Matrix R1 = Matrix_(2,2, data1);
  Vector d1(2); d1(0) = -1; d1(1) = -1;
  ConditionalGaussian::shared_ptr cg1(new ConditionalGaussian(d1, R1));
  
  double data21[] = { 6.7082, 0.0,
                     0.0, 6.7082};
  Matrix R2 = Matrix_(2,2, data21);
  double data22[] = { -6.7082, 0.0,
                       0.0, -6.7082};
  Matrix A1 = Matrix_(2,2, data22);
  Vector d2(2); d2(0) = 0.0; d2(1) = 1.34164;
  ConditionalGaussian::shared_ptr cg2(new ConditionalGaussian(d2, R2, "x1", A1));
  
  double data31[] = { 11.1803, 0.0,
                         0.0, 11.1803};
  Matrix R3 = Matrix_(2,2, data31);
  double data32[] = { -2.23607, 0.0,
                          0.0, -2.23607};
  Matrix A21 = Matrix_(2,2, data32);
  double data33[] = { -8.94427, 0.0,
                          0.0, -8.94427};
  Matrix A22 = Matrix_(2,2, data33);
  
  Vector d3(2); d3(0) = 2.23607; d3(1) = -1.56525;
  ConditionalGaussian::shared_ptr cg3(new ConditionalGaussian(d3, R3, "l1", A21, "x1", A22));
  
  ChordalBayesNet expected;
  expected.insert("x1", cg1);
  expected.insert("l1", cg2);
  expected.insert("x2", cg3);
  
  // Check one ordering
  LinearFactorGraph fg1 = createLinearFactorGraph();
  Ordering ord1;
  ord1.push_back("x2");
  ord1.push_back("l1");
  ord1.push_back("x1");
  ChordalBayesNet::shared_ptr actual = fg1.eliminate(ord1);
  CHECK(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, add_priors )
{
  LinearFactorGraph fg = createLinearFactorGraph();
  LinearFactorGraph actual = fg.add_priors(3);
  LinearFactorGraph expected = createLinearFactorGraph();
  Matrix A = 3*eye(2);
  Vector b = zero(2);
  expected.push_back(LinearFactor::shared_ptr(new LinearFactor("l1",A,b)));
  expected.push_back(LinearFactor::shared_ptr(new LinearFactor("x1",A,b)));
  expected.push_back(LinearFactor::shared_ptr(new LinearFactor("x2",A,b)));
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, copying )
{
  // Create a graph
  LinearFactorGraph actual = createLinearFactorGraph();

  // Copy the graph !
  LinearFactorGraph copy = actual;

  // now eliminate the copy
  Ordering ord1;
  ord1.push_back("x2");
  ord1.push_back("l1");
  ord1.push_back("x1");
  ChordalBayesNet::shared_ptr actual1 = copy.eliminate(ord1);

  // Create the same graph, but not by copying
  LinearFactorGraph expected = createLinearFactorGraph();

  // and check that original is still the same graph
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, matrix )
{
  // Create a graph
  LinearFactorGraph fg = createLinearFactorGraph();

  // render with a given ordering
  Ordering ord;
  ord.push_back("x2");
  ord.push_back("l1");
  ord.push_back("x1");

  Matrix A; Vector b;
  boost::tie(A,b) = fg.matrix(ord);

  Matrix A1 = Matrix_(2*4,3*2,
		     00.0,  0.0,  0.0,  0.0, 10.0,  0.0,
		     00.0,  0.0,  0.0,  0.0,  0.0, 10.0,
		     10.0,  0.0,  0.0,  0.0,-10.0,  0.0,
		     00.0, 10.0,  0.0,  0.0,  0.0,-10.0,
		     00.0,  0.0,  5.0,  0.0, -5.0,  0.0,
		     00.0,  0.0,  0.0,  5.0,  0.0, -5.0,
		     -5.0,  0.0,  5.0,  0.0,  0.0,  0.0,
		     00.0, -5.0,  0.0,  5.0,  0.0,  0.0
    );
  Vector b1 = Vector_(8,-1.0, -1.0, 2.0, -1.0, 0.0, 1.0, -1.0, 1.5);

  EQUALITY(A,A1); // currently fails
  CHECK(b==b1); // currently fails
}

/* ************************************************************************* */
TEST( LinearFactorGraph, CONSTRUCTOR_ChordalBayesNet )
{

  LinearFactorGraph fg = createLinearFactorGraph();

  // render with a given ordering
  Ordering ord;
  ord.push_back("x2");
  ord.push_back("l1");
  ord.push_back("x1");
  
  ChordalBayesNet::shared_ptr CBN = fg.eliminate(ord);
  LinearFactorGraph fg2(*CBN);
  ChordalBayesNet::shared_ptr CBN2 = fg2.eliminate(ord);
  
  CHECK(CBN->equals(*CBN2));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, GET_ORDERING)
{
  LinearFactorGraph fg = createLinearFactorGraph();
  Ordering ord = fg.getOrdering();
  CHECK(ord[0] == string("l1"));
  CHECK(ord[1] == string("x1"));
  CHECK(ord[2] == string("x2"));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, OPTIMIZE )
{
	// create a graph
	LinearFactorGraph fg = createLinearFactorGraph();

	// create an ordering
	Ordering ord = fg.getOrdering();

	// optimize the graph
	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected = createCorrectDelta();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, COMBINE_GRAPHS_INPLACE)
{
		// create a test graph 
		LinearFactorGraph fg1 = createLinearFactorGraph();
		
		// create another factor graph
		LinearFactorGraph fg2 = createLinearFactorGraph();
		
		// get sizes
		int size1 = fg1.size();
		int size2 = fg2.size();
		
		// combine them
		fg1.combine(fg2);
		
		CHECK(size1+size2 == fg1.size());  
}

/* ************************************************************************* */
TEST( LinearFactorGraph, COMBINE_GRAPHS)
{
		// create a test graph 
		LinearFactorGraph fg1 = createLinearFactorGraph();
		
		// create another factor graph
		LinearFactorGraph fg2 = createLinearFactorGraph();
		
		// get sizes
		int size1 = fg1.size();
		int size2 = fg2.size();
		
		// combine them
		LinearFactorGraph fg3 = LinearFactorGraph::combine2(fg1, fg2);
		
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
TEST( LinearFactorGraph, factor_lookup)
{
	// create a test graph
	LinearFactorGraph fg = createLinearFactorGraph();

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
TEST( LinearFactorGraph, findAndRemoveFactors )
{
	// create the graph
	LinearFactorGraph fg = createLinearFactorGraph();

  // We expect to remove these three factors: 0, 1, 2
  LinearFactor::shared_ptr f0 = fg[0];
  LinearFactor::shared_ptr f1 = fg[1];
  LinearFactor::shared_ptr f2 = fg[2];

  // call the function
  vector<LinearFactor::shared_ptr> factors = fg.findAndRemoveFactors("x1");

  // Check the factors
  CHECK(f0==factors[0]);
  CHECK(f1==factors[1]);
  CHECK(f2==factors[2]);

  // CHECK if the factors are deleted from the factor graph
  LONGS_EQUAL(1,fg.nrFactors());
  }

/* ************************************************************************* */
TEST( LinearFactorGraph, findAndRemoveFactors_twice )
{
	// create the graph
	LinearFactorGraph fg = createLinearFactorGraph();

  // We expect to remove these three factors: 0, 1, 2
  LinearFactor::shared_ptr f0 = fg[0];
  LinearFactor::shared_ptr f1 = fg[1];
  LinearFactor::shared_ptr f2 = fg[2];

  // call the function
  vector<LinearFactor::shared_ptr> factors = fg.findAndRemoveFactors("x1");

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
TEST(timeLinearFactorGraph, createSmoother)
{
	LinearFactorGraph fg1 = createSmoother(1);
	LONGS_EQUAL(3,fg1.size());
	LinearFactorGraph fg2 = createSmoother(2);
	LONGS_EQUAL(5,fg2.size());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
