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

// TODO: DANGEROUS, create shared pointers
#define GTSAM_MAGIC_GAUSSIAN 2
#define GTSAM_MAGIC_KEY

#include "Matrix.h"
#include "smallExample.h"
#include "simulated2D.h"
#include "GaussianFactor.h"

using namespace std;
using namespace gtsam;
using namespace example;

typedef boost::shared_ptr<NonlinearFactor<VectorConfig> > shared_nlf;

/* ************************************************************************* */
TEST( NonlinearFactor, equals )
{
	SharedGaussian sigma(noiseModel::Isotropic::Sigma(2,1.0));

	// create two nonlinear2 factors
	Point2 z3(0.,-1.);
	simulated2D::Measurement f0(z3, sigma, 1,1);

	// measurement between x2 and l1
	Point2 z4(-1.5, -1.);
	simulated2D::Measurement f1(z4, sigma, 2,1);

	CHECK(assert_equal(f0,f0));
	CHECK(f0.equals(f0));
	CHECK(!f0.equals(f1));
	CHECK(!f1.equals(f0));
}

/* ************************************************************************* */
TEST( NonlinearFactor, equals2 )
{
  // create a non linear factor graph
  Graph fg = createNonlinearFactorGraph();

  // get two factors
  Graph::sharedFactor f0 = fg[0], f1 = fg[1];

  CHECK(f0->equals(*f0));
  CHECK(!f0->equals(*f1));
  CHECK(!f1->equals(*f0));
}

/* ************************************************************************* */
TEST( NonlinearFactor, NonlinearFactor )
{
  // create a non linear factor graph
  Graph fg = createNonlinearFactorGraph();

  // create a configuration for the non linear factor graph
  Config cfg = createNoisyConfig();

  // get the factor "f1" from the factor graph
  Graph::sharedFactor factor = fg[0];

  // calculate the error_vector from the factor "f1"
  // the expected value for the whitened error from the factor
  // error_vector / sigma = [0.1 0.1]/0.1 = [1;1]
  Vector actual_e = factor->whitenedError(cfg);
  CHECK(assert_equal(ones(2),actual_e));

  // error = 0.5 * [1 1] * [1;1] = 1
  double expected = 1.0; 

  // calculate the error from the factor "f1"
  double actual = factor->error(cfg);
  DOUBLES_EQUAL(expected,actual,0.00000001);
}

/* ************************************************************************* *
TEST( NonlinearFactor, linearize_f1 )
{
  // Grab a non-linear factor
  Graph nfg = createNonlinearFactorGraph();
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

/* ************************************************************************* *
TEST( NonlinearFactor, linearize_f2 )
{
  // Grab a non-linear factor
  Graph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[1]);

  // We linearize at noisy config from SmallExample
  VectorConfig c = createNoisyConfig();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[1];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* *
TEST( NonlinearFactor, linearize_f3 )
{
  // Grab a non-linear factor
  Graph nfg = createNonlinearFactorGraph();
  boost::shared_ptr<NonlinearFactor1> nlf = 
    boost::static_pointer_cast<NonlinearFactor1>(nfg[2]);

  // We linearize at noisy config from SmallExample
  VectorConfig c = createNoisyConfig();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[2];

  CHECK(expected->equals(*actual));
}

/* ************************************************************************* *
TEST( NonlinearFactor, linearize_f4 )
{
  // Grab a non-linear factor
  Graph nfg = createNonlinearFactorGraph();
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
	Graph fg = createNonlinearFactorGraph();
	
	// create a configuration for the non linear factor graph
	Config cfg = createNoisyConfig();
	
	// get some factors from the graph
	Graph::sharedFactor factor1 = fg[0], factor2 = fg[1],
			factor3 = fg[2];
	
	CHECK(factor1->size() == 1);
	CHECK(factor2->size() == 2);
	CHECK(factor3->size() == 2);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
