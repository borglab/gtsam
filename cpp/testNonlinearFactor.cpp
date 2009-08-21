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

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( NonLinearFactor, NonlinearFactor )
{
  // create a non linear factor graph
  NonlinearFactorGraph fg = createNonlinearFactorGraph();

  // create a configuration for the non linear factor graph
  FGConfig cfg = createNoisyConfig();

  // get the factor "f1" from the factor graph
  boost::shared_ptr<NonlinearFactor> factor = fg[0];

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
TEST( NonLinearFactor, linearize_f1 )
{
  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[0]);

  // We linearize at noisy config from SmallExample
  FGConfig c = createNoisyConfig();
  LinearFactor::shared_ptr actual = nlf->linearize(c);

  LinearFactorGraph lfg = createLinearFactorGraph();
  LinearFactor::shared_ptr expected = lfg[0];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* */
TEST( NonLinearFactor, linearize_f2 )
{
  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[1]);

  // We linearize at noisy config from SmallExample
  FGConfig c = createNoisyConfig();
  LinearFactor::shared_ptr actual = nlf->linearize(c);

  LinearFactorGraph lfg = createLinearFactorGraph();
  LinearFactor::shared_ptr expected = lfg[1];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* */
TEST( NonLinearFactor, linearize_f3 )
{
  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[2]);

  // We linearize at noisy config from SmallExample
  FGConfig c = createNoisyConfig();
  LinearFactor::shared_ptr actual = nlf->linearize(c);

  LinearFactorGraph lfg = createLinearFactorGraph();
  LinearFactor::shared_ptr expected = lfg[2];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* */
TEST( NonLinearFactor, linearize_f4 )
{
  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[3]);

  // We linearize at noisy config from SmallExample
  FGConfig c = createNoisyConfig();
  LinearFactor::shared_ptr actual = nlf->linearize(c);

  LinearFactorGraph lfg = createLinearFactorGraph();
  LinearFactor::shared_ptr expected = lfg[3];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* */
TEST( NonLinearFactor, size )
{
	// create a non linear factor graph
	NonlinearFactorGraph fg = createNonlinearFactorGraph();
	
	// create a configuration for the non linear factor graph
	FGConfig cfg = createNoisyConfig();
	
	// get some factors from the graph
	boost::shared_ptr<NonlinearFactor> factor1 = fg[0];
	boost::shared_ptr<NonlinearFactor> factor2 = fg[1];
	boost::shared_ptr<NonlinearFactor> factor3 = fg[2];
	
	CHECK(factor1->size() == 1);
	CHECK(factor2->size() == 2);
	CHECK(factor3->size() == 2);
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
