/**
 *  @file  testNonlinearFactor.cpp
 *  @brief Unit tests for Non-Linear Factor, 
 *  create a non linear factor graph and a configuration for it and
 *  calculate the error for the factor.
 *  @author Christian Potthast
 **/

/*STL/C++*/
#include <iostream>

#include <CppUnitLite/TestHarness.h>

#include "Matrix.h"
#include "smallExample.h"
#include "Simulated2DMeasurement.h"
#include "GaussianFactor.h"

using namespace std;
using namespace gtsam;

typedef boost::shared_ptr<NonlinearFactor<VectorConfig> > shared_nlf;

/* ************************************************************************* */
TEST( NonlinearFactor, equals )
{
	double sigma = 1.0;

	// create two nonlinear2 factors
	Vector z3(2); z3(0) = 0. ; z3(1) = -1.;
	Simulated2DMeasurement f0(z3, sigma, "x1", "l1");

	// measurement between x2 and l1
	Vector z4(2); z4(0)= -1.5 ; z4(1) = -1.;
	Simulated2DMeasurement f1(z4, sigma, "x2", "l1");

	CHECK(assert_equal(f0,f0));
	CHECK(f0.equals(f0));
	CHECK(!f0.equals(f1));
	CHECK(!f1.equals(f0));
}

/* ************************************************************************* */
TEST( NonlinearFactor, equals2 )
{
  // create a non linear factor graph
  ExampleNonlinearFactorGraph fg = createNonlinearFactorGraph();

  // get two factors
  shared_nlf f0 = fg[0], f1 = fg[1];

  CHECK(f0->equals(*f0));
  CHECK(!f0->equals(*f1));
  CHECK(!f1->equals(*f0));
}

/* ************************************************************************* */
TEST( NonlinearFactor, NonlinearFactor )
{
  // create a non linear factor graph
  ExampleNonlinearFactorGraph fg = createNonlinearFactorGraph();

  // create a configuration for the non linear factor graph
  VectorConfig cfg = createNoisyConfig();

  // get the factor "f1" from the factor graph
  shared_nlf factor = fg[0];

  // calculate the error_vector from the factor "f1"
  Vector actual_e = factor->error_vector(cfg);
  Vector e(2); e(0) = -0.1;  e(1) = -0.1;
  CHECK(assert_equal(e,actual_e));

  // the expected value for the error from the factor
  // error_vector / sigma = [0.1 0.1]/0.1 = [1;1]
  // error = 0.5 * [1 1] * [1;1] = 1
  double expected = 1.0; 

  // calculate the error from the factor "f1"
  double actual = factor->error(cfg);
  DOUBLES_EQUAL(expected,actual,0.00000001);
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f1 )
{
  // Grab a non-linear factor
  ExampleNonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[0]);

  // We linearize at noisy config from SmallExample
  VectorConfig c = createNoisyConfig();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[0];

  CHECK(assert_equal(*expected,*actual));

  // The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
  // Hence i.e., b = approximates z-h(x0) = error_vector(x0)
	CHECK(assert_equal(nlf->error_vector(c),actual->get_b()));
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f2 )
{
  // Grab a non-linear factor
  ExampleNonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[1]);

  // We linearize at noisy config from SmallExample
  VectorConfig c = createNoisyConfig();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[1];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f3 )
{
  // Grab a non-linear factor
  ExampleNonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[2]);

  // We linearize at noisy config from SmallExample
  VectorConfig c = createNoisyConfig();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[2];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f4 )
{
  // Grab a non-linear factor
  ExampleNonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[3]);

  // We linearize at noisy config from SmallExample
  VectorConfig c = createNoisyConfig();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[3];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, size )
{
	// create a non linear factor graph
	ExampleNonlinearFactorGraph fg = createNonlinearFactorGraph();
	
	// create a configuration for the non linear factor graph
	VectorConfig cfg = createNoisyConfig();
	
	// get some factors from the graph
	shared_nlf factor1 = fg[0];
	shared_nlf factor2 = fg[1];
	shared_nlf factor3 = fg[2];
	
	CHECK(factor1->size() == 1);
	CHECK(factor2->size() == 2);
	CHECK(factor3->size() == 2);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
