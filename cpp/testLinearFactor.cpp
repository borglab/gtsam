/**
 *  @file   testLinearFactor.cpp
 *  @brief  Unit tests for Linear Factor
 *  @author Christian Potthast
 *  @author Frank Dellaert
 **/

#include <iostream>

#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Matrix.h"
#include "smallExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( LinearFactor, linearFactor )
{	
    Matrix A1(2,2);
    A1(0,0) = -10.0 ; A1(0,1) = 0.0;
    A1(1,0) = 0.0 ; A1(1,1) = -10.0;

    Matrix A2(2,2);
    A2(0,0) = 10.0 ; A2(0,1) = 0.0;
    A2(1,0) = 0.0 ; A2(1,1) = 10.0;   
    
    Vector b(2);
    b(0) = 2 ; b(1) = -1;
    
    LinearFactor expected("x1", A1,  "x2", A2, b);

    // create a small linear factor graph
    LinearFactorGraph fg = createLinearFactorGraph();

    // get the factor "f2" from the factor graph
    LinearFactor::shared_ptr lf = fg[1];

    // check if the two factors are the same
    CHECK(assert_equal(expected,*lf));
}

/* ************************************************************************* */
TEST( LinearFactor, keys )
{
  // get the factor "f2" from the small linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();
  LinearFactor::shared_ptr lf = fg[1];
  list<string> expected;
  expected.push_back("x1");
  expected.push_back("x2");
  CHECK(lf->keys() == expected);
}

/* ************************************************************************* */
TEST( LinearFactor, variables )
{	
  // get the factor "f2" from the small linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();
  LinearFactor::shared_ptr lf = fg[1];
  VariableSet vs = lf->variables();
}

/* ************************************************************************* */
TEST( LinearFactor, linearFactor2 )
{
  // create a small linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();

  // get two factors from it and insert the factors into a set
  vector<LinearFactor::shared_ptr> lfg;
  lfg.push_back(fg[4 - 1]);
  lfg.push_back(fg[2 - 1]);

  // combine in a factor
  LinearFactor combined(lfg);

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
  b2(0) = -1;
  b2(1) = 1.5;
  b2(2) = 2;
  b2(3) = -1;

  LinearFactor expected("x2", Ax2,  "l1", Al1, "x1", Ax1, b2);
  CHECK(assert_equal(expected,combined));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, linearFactor3){

	Matrix A11(2,2);
	A11(0,0) = 10.4545; A11(0,1) =  0;
	A11(1,0) = 0;       A11(1,1) = 10.4545;
	Vector b(2);
	b(0) = 2 ; b(1) = -1;
	LinearFactor::shared_ptr f1(new LinearFactor("x1", A11, b));

	A11(0,0) = 2; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -2;
	b(0) = 4 ; b(1) = -5;
	LinearFactor::shared_ptr f2(new LinearFactor("x1", A11, b));

	A11(0,0) = 4; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -4;
	b(0) = 3 ; b(1) = -88;
	LinearFactor::shared_ptr f3(new LinearFactor("x1", A11, b));

	A11(0,0) = 6; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = 7;
	b(0) = 5 ; b(1) = -6;
	LinearFactor::shared_ptr f4(new LinearFactor("x1", A11, b));

	vector<LinearFactor::shared_ptr> lfg;
	lfg.push_back(f1);
	lfg.push_back(f2);
	lfg.push_back(f3);
	lfg.push_back(f4);
	LinearFactor combined(lfg);

	Matrix A22(8,2);
	A22(0,0) = 10.4545; A22(0,1) =  0;
	A22(1,0) = 0;       A22(1,1) = 10.4545;
	A22(2,0) = 2;       A22(2,1) =  0;
	A22(3,0) = 0;       A22(3,1) = -2;
	A22(4,0) = 4;       A22(4,1) =  0;
	A22(5,0) = 0;       A22(5,1) = -4;
	A22(6,0) = 6;       A22(6,1) =  0;
	A22(7,0) = 0;       A22(7,1) =  7;
	Vector exb(8);
	exb(0) = 2 ; exb(1) = -1;  exb(2) = 4 ; exb(3) = -5;
	exb(4) = 3 ; exb(5) = -88; exb(6) = 5 ; exb(7) = -6;
	LinearFactor expected("x1", A22, exb);

	CHECK(assert_equal(expected,combined));
}

/* ************************************************************************* */
TEST( LinearFactor, linearFactorN){
  vector<LinearFactor::shared_ptr> f;
  f.push_back(LinearFactor::shared_ptr(new LinearFactor("x1", Matrix_(2,2,
      1.0, 0.0,
      0.0, 1.0),
      Vector_(2,
      10.0, 5.0))));
  f.push_back(LinearFactor::shared_ptr(new LinearFactor("x1", Matrix_(2,2,
      -10.0, 0.0,
      0.0, -10.0),
      "x2", Matrix_(2,2,
      10.0, 0.0,
      0.0, 10.0),
      Vector_(2,
      1.0, -2.0))));
  f.push_back(LinearFactor::shared_ptr(new LinearFactor("x2", Matrix_(2,2,
      -10.0, 0.0,
      0.0, -10.0),
      "x3", Matrix_(2,2,
      10.0, 0.0,
      0.0, 10.0),
      Vector_(2,
      1.5, -1.5))));
  f.push_back(LinearFactor::shared_ptr(new LinearFactor("x3", Matrix_(2,2,
      -10.0, 0.0,
      0.0, -10.0),
      "x4", Matrix_(2,2,
      10.0, 0.0,
      0.0, 10.0),
      Vector_(2,
      2.0, -1.0))));

  LinearFactor combinedFactor(f);

  vector<pair<string, Matrix> > combinedMeasurement;
  combinedMeasurement.push_back(make_pair("x1", Matrix_(8,2,
      1.0, 0.0,
      0.0, 1.0,
      -10.0, 0.0,
      0.0, -10.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0)));
  combinedMeasurement.push_back(make_pair("x2", Matrix_(8,2,
      0.0, 0.0,
      0.0, 0.0,
      10.0, 0.0,
      0.0, 10.0,
      -10.0, 0.0,
      0.0, -10.0,
      0.0, 0.0,
      0.0, 0.0)));
  combinedMeasurement.push_back(make_pair("x3", Matrix_(8,2,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      10.0, 0.0,
      0.0, 10.0,
      -10.0, 0.0,
      0.0, -10.0)));
  combinedMeasurement.push_back(make_pair("x4", Matrix_(8,2,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      10.0, 0.0,
      0.0, 10.0)));
  Vector b = Vector_(8,
      10.0, 5.0, 1.0, -2.0, 1.5, -1.5, 2.0, -1.0);

  LinearFactor expected(combinedMeasurement, b);
  CHECK(combinedFactor.equals(expected));
}

/* ************************************************************************* */
TEST( LinearFactor, error )
{	
    // create a small linear factor graph
    LinearFactorGraph fg = createLinearFactorGraph();

    // get the first factor from the factor graph
    LinearFactor::shared_ptr lf = fg[0];

    // check the error of the first factor with nosiy config
    VectorConfig cfg = createZeroDelta();

    // calculate the error from the factor "f1"
    // note the error is the same as in testNonlinearFactor
    double actual = lf->error(cfg);
    DOUBLES_EQUAL( 1.0, actual, 0.00000001 );
}

/* ************************************************************************* */
TEST( LinearFactor, eliminate )
{	
  // the combined linear factor
  Matrix Ax2 = Matrix_(4,2,
			 // x2  
			 -5., 0.,
			 +0.,-5.,
			 10., 0.,
			 +0.,10.
			 );
                     
  Matrix Al1 = Matrix_(4,2,
			 // l1     
			 5., 0.,
			 0., 5.,
			 0., 0.,
			 0., 0.
			 );
                     
  Matrix Ax1 = Matrix_(4,2,
                         // x1
			 0.00,  0., // f4
			 0.00,  0., // f4
			 -10.,  0., // f2
			 0.00,-10.  // f2
			 );

  // the RHS
  Vector b2(4);
  b2(0) = -1;
  b2(1) = 1.5;
  b2(2) = 2;
  b2(3) = -1;
  
  LinearFactor combined("x2", Ax2,  "l1", Al1, "x1", Ax1, b2);

  // eliminate the combined factor

  ConditionalGaussian::shared_ptr actualCG;
  LinearFactor::shared_ptr actualLF;
  boost::tie(actualCG,actualLF) = combined.eliminate("x2");

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
  ConditionalGaussian expectedCG(d,R11,"l1",S12,"x1",S13);

  // the expected linear factor
  Matrix Bl1 = Matrix_(2,2,
			 // l1     
			 4.47214, 0.00,
			 0.00, 4.47214
			 );
                     
  Matrix Bx1 = Matrix_(2,2,
                         // x1
			 -4.47214,  0.00,
			 +0.00, -4.47214
			 );

  // the RHS
  Vector b1(2); b1(0) = 0.0; b1(1) = 0.894427;
  
  LinearFactor expectedLF("l1", Bl1, "x1", Bx1, b1);

  // check if the result matches
  CHECK(assert_equal(expectedCG,*actualCG,1e-4));
  CHECK(assert_equal(expectedLF,*actualLF,1e-5));
}


/* ************************************************************************* */
TEST( LinearFactor, eliminate2 )
{	
  // the combined linear factor
  Matrix Ax2 = Matrix_(4,2,
			 // x2  
			 -5., 0.,
			 +0.,-5.,
			 10., 0.,
			 +0.,10.
			 );
                     
  Matrix Al1x1 = Matrix_(4,4,
			   // l1   x1   
			   5., 0., 0.00,  0., // f4
			   0., 5., 0.00,  0., // f4
			   0., 0., -10.,  0., // f2
			   0., 0., 0.00,-10.  // f2
			   );
                     
  // the RHS
  Vector b2(4);
  b2(0) = -1;
  b2(1) = 1.5;
  b2(2) = 2;
  b2(3) = -1;
  
  LinearFactor combined("x2", Ax2,  "l1x1", Al1x1, b2);

  // eliminate the combined factor
  ConditionalGaussian::shared_ptr actualCG;
  LinearFactor::shared_ptr actualLF;
  boost::tie(actualCG,actualLF) = combined.eliminate("x2");

  // create expected Conditional Gaussian
  Matrix R11 = Matrix_(2,2,
			 11.1803,  0.00,
			 0.00, 11.1803
			 );
  Matrix S12 = Matrix_(2,4,
			 -2.23607, 0.00,-8.94427, 0.00,
			 +0.00,-2.23607,+0.00,-8.94427
			 );
  Vector d(2); d(0) = 2.23607; d(1) = -1.56525;
  ConditionalGaussian expectedCG(d,R11,"l1x1",S12);

  // the expected linear factor
  Matrix Bl1x1 = Matrix_(2,4,
			   // l1          x1
			   4.47214, 0.00, -4.47214,  0.00,
			   0.00, 4.47214, +0.00, -4.47214
			   );
                     
  // the RHS
  Vector b1(2); b1(0) = 0.0; b1(1) = 0.894427;
  
  LinearFactor expectedLF("l1x1", Bl1x1, b1);

  // check if the result matches
  CHECK(assert_equal(expectedCG,*actualCG,1e-4));
  CHECK(assert_equal(expectedLF,*actualLF,1e-5));
}

//* ************************************************************************* */
TEST( LinearFactor, default_error )
{	
  LinearFactor f;
  VectorConfig c;
  double actual = f.error(c);
  CHECK(actual==0.0);
}

//* ************************************************************************* */
TEST( LinearFactor, eliminate_empty )
{	
  // create an empty factor
  LinearFactor f;

  // eliminate the empty factor
  ConditionalGaussian::shared_ptr actualCG;
  LinearFactor::shared_ptr actualLF;
  boost::tie(actualCG,actualLF) = f.eliminate("x2");

  // expected Conditional Gaussian is just a parent-less node with P(x)=1
  ConditionalGaussian expectedCG;

  // expected remaining factor is still empty :-)
  LinearFactor expectedLF;

  // check if the result matches
  CHECK(actualCG->equals(expectedCG));
  CHECK(actualLF->equals(expectedLF));
}

//* ************************************************************************* */
TEST( LinearFactor, empty )
{	
  // create an empty factor
  LinearFactor f;
  CHECK(f.empty()==true);
}

/* ************************************************************************* */
TEST( LinearFactor, matrix )
{
  // create a small linear factor graph
  LinearFactorGraph fg = createLinearFactorGraph();

  // get the factor "f2" from the factor graph
  LinearFactor::shared_ptr lf = fg[1];

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
TEST( LinearFactor, size )
{
	// create a linear factor graph
	LinearFactorGraph fg = createLinearFactorGraph();
	
	// get some factors from the graph
	boost::shared_ptr<LinearFactor> factor1 = fg[0];
	boost::shared_ptr<LinearFactor> factor2 = fg[1];
	boost::shared_ptr<LinearFactor> factor3 = fg[2];
	
	CHECK(factor1->size() == 1);
	CHECK(factor2->size() == 2);
	CHECK(factor3->size() == 2);
}

/* ************************************************************************* */
TEST( LinearFactor, CONSTRUCTOR_ConditionalGaussian )
{
  Matrix R11 = Matrix_(2,2,
			 11.1803,  0.00,
			 0.00, 11.1803
			 );
  Matrix S12 = Matrix_(2,2,
		       -2.23607, 0.00,
		       +0.00,-2.23607
		       );
  Vector d(2); d(0) = 2.23607; d(1) = -1.56525;
  ConditionalGaussian::shared_ptr CG(new ConditionalGaussian(d,R11,"l1x1",S12) );
  LinearFactor actualLF("x2",CG);
  //  actualLF.print();
  LinearFactor expectedLF("x2",R11,"l1x1",S12,d);
			  
  CHECK(assert_equal(expectedLF,actualLF));
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
