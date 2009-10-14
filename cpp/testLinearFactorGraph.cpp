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
#include "FactorGraph-inl.h" // template definitions

using namespace gtsam;

/* ************************************************************************* */
/* unit test for equals (LinearFactorGraph1 == LinearFactorGraph2)           */ 
/* ************************************************************************* */
TEST( LinearFactorGraph, equals ){

  LinearFactorGraph fg = createLinearFactorGraph();
  LinearFactorGraph fg2 = createLinearFactorGraph();
  CHECK( fg.equals(fg2) );
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
/* unit test for find factors                                                */ 
/* ************************************************************************* */
/*
TEST( LinearFactorGraph, find_factors )
{	
  int checksum = 0;
  int expected = 16;

  LinearFactorGraph fg = createLinearFactorGraph();

  // create a shared pointer object to prevent memory leaks
  set<shared_ptr<LinearFactor> > factors = fg.find_factors_and_remove("x2");
  
  // CHECK whether find the right two factors
  
  // odometry between x1 and x2
  Matrix A21(2,2);
  A21(0,0) = -10.0 ; A21(0,1) = 0.0;
  A21(1,0) = 0.0   ; A21(1,1) = -10.0;

  Matrix A22(2,2);
  A22(0,0) = 10.0 ; A22(0,1) = 0.0;
  A22(1,0) = 0.0  ; A22(1,1) = 10.0;   
  
  Vector b(2);
  b(0) = 2 ; b(1) = -1;
  
  LinearFactor f2 = LinearFactor("x1", A21,  "x2", A22, b);

  // measurement between x2 and l1
  Matrix A41(2,2);
  A41(0,0) = -5  ; A41(0,1) = 0.0;
  A41(1,0) = 0.0  ; A41(1,1) = -5;
  
  Matrix A42(2,2);
  A42(0,0) = 5 ; A42(0,1) = 0;
  A42(1,0) = 0 ; A42(1,1) = 5;      

  b(0)= -1 ; b(1) = 1.5;

  LinearFactor f4 = LinearFactor("x2", A41, "l1", A42, b);

  set<shared_ptr<LinearFactor> >::iterator it;
  for( it = factors.begin() ; it != factors.end() ; it++ ){
    if( strcmp( it->get()->name_.c_str(), "f2") == 0){
      if(it->get()->equals(f2)){
        checksum = checksum + 2;  
      }
    }else if( strcmp( it->get()->name_.c_str(), "f4") == 0){
      if(it->get()->equals(f4)){
        checksum = checksum + 4;  
      }else{
      }
    }
  }
 
  // CHECK if the factors are deleted from the factor graph
  
  // Create
  LinearFactorGraph fg2;
  
  // prior on x1
  Matrix A11(2,2);
  A11(0,0) = 10 ; A11(0,1) = 0;
  A11(1,0) = 0  ; A11(1,1) = 10;
  
  b(0) = -1 ; b(1) = -1;
  
  LinearFactor *f1 = new LinearFactor("x1", A11, b);
  fg2.push_back(f1);
  
  // measurement between x1 and l1
  Matrix A31(2,2);
  A31(0,0) = -5 ; A31(0,1) = 0;
  A31(1,0) = 0  ; A31(1,1) = -5;
  
  Matrix A32(2,2);
  A32(0,0) = 5 ; A32(0,1) = 0;
  A32(1,0) = 0 ; A32(1,1) = 5;  

  b(0) = 0 ; b(1) = 1;

  LinearFactor *f3 = new LinearFactor("x1", A31, "l1", A32, b);
  fg2.push_back(f3);

  if( fg.equals(fg2) ){
    checksum = checksum + 10; 
  }

  CHECK( checksum == expected );    
}
*/

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
// Note: This test does not pass when running on Linux(Ubuntu 9.04, GCC 4.3.3) 
//       systems.  
TEST( LinearFactorGraph, combine_factors_x1 )
{	
  // create a small example for a linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();

  // combine all factors
  LinearFactor::shared_ptr actual = fg.combine_factors("x1");

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
  CHECK(actual->equals(expected));  //currently fails
}

/* ************************************************************************* */
TEST( LinearFactorGraph, combine_factors_x2 )
{	
 // create a small example for a linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();

  // combine all factors
  LinearFactor::shared_ptr actual = fg.combine_factors("x2");

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
  CHECK(actual->equals(expected)); // currently fails - ordering is different
}

/* ************************************************************************* */

TEST( LinearFactorGraph, eliminate_one_x1 )
{
  LinearFactorGraph fg = createLinearFactorGraph();
  ConditionalGaussian::shared_ptr actual = fg.eliminate_one("x1");

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

  CHECK( actual->equals(expected) );
}

/* ************************************************************************* */
 
TEST( LinearFactorGraph, eliminate_one_x2 )
{
  LinearFactorGraph fg = createLinearFactorGraph();
  ConditionalGaussian::shared_ptr actual = fg.eliminate_one("x2");

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

  CHECK( actual->equals(expected) );
}

/* ************************************************************************* */
TEST( LinearFactorGraph, eliminate_one_l1 )
{
  LinearFactorGraph fg = createLinearFactorGraph();
  ConditionalGaussian::shared_ptr actual = fg.eliminate_one("l1");

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

  CHECK( actual->equals(expected) );
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
  ChordalBayesNet::shared_ptr actual1 = fg1.eliminate(ord1);
  CHECK(actual1->equals(expected));
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
  CHECK(actual.equals(expected));
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
  CHECK(actual.equals(expected));
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

	CHECK(actual.equals(expected));
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
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
