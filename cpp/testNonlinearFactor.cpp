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
#include "Pose2.h"

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

  CHECK(expected->equals(*actual));
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
Vector RotatePoseDisplacement(Vector d, double theta) {
	double co=cos(theta);
	double si=sin(theta);
	return Matrix_(3,3, co, -si, 0.0, si, co, 0.0, 0.0, 0.0, 1.0)*d;


}
Vector h(const Pose2& p1, const Pose2& p2) {
	double dx= p2.x()-p1.x();
	double dy= p2.y()-p1.y();
	double dtheta= p2.theta()-p1.theta();
	return RotatePoseDisplacement(Vector_(3,dx,dy,dtheta),-p1.theta());
}

Matrix H1(const Pose2& p1, const Pose2& p2) {

	double dx= p2.x()-p1.x();
	double dy= p2.y()-p1.y();
	double co=cos(p1.theta());
	double si=sin(p1.theta());
	return Matrix_(3,3, -co, -si, -si*dx+co*dy, si, -co, -co*dx-si*dy, 0.0, 0.0, -1.0);

}

Matrix H2(const Pose2& p1) {
	double co=cos(p1.theta());
	double si=sin(p1.theta());
	return Matrix_(3,3, co, si, 0.0, -si, co, 0.0, 0.0, 0.0, 1.0);
}

TEST( PoseConstraintFactor2, testFunctions )
{
	Pose2 p1(0.0, 6.0, 0.0);
	Pose2 p2(0.101826, 6.111236, 0.011499);
	//expected
	Vector expectedh = Vector_(3, 0.101826, 0.111236, 0.011499);
	Matrix expectedH1 = Matrix_(3,3,-1.0, 0.0, 0.111236, 0.0, -1.0, -0.101826, 0.0, 0.0, -1.0);
	Matrix expectedH2 = Matrix_(3,3, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

	// actual
	Vector actualh = h(p1,p2);
	Matrix actualH1 = H1(p1,p2);
	Matrix actualH2 = H2(p1);

	CHECK(assert_equal(actualh,expectedh));
	CHECK(assert_equal(actualH1,expectedH1));
	CHECK(assert_equal(actualH2,expectedH2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
