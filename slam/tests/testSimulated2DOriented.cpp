/*
 * testSimulated2DOriented.cpp
 *
 *   Created on: Jun 10, 2010
 *       Author: nikai
 *  Description: unit tests for simulated2DOriented
 */

#include <iostream>
#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/slam/Simulated2DConfig.h>
#include <gtsam/slam/simulated2DOriented.h>
#include <gtsam/slam/Simulated2DOrientedOdometry.h>

using namespace gtsam;
using namespace std;
using namespace simulated2DOriented;

/* ************************************************************************* */
TEST( simulated2DOriented, Simulated2DConfig )
{
	Simulated2DConfig actual;
	actual.insertPose(1,Point2(1,1));
	actual.insertPoint(2,Point2(2,2));
  CHECK(assert_equal(actual,actual,1e-9));
}

/* ************************************************************************* */
TEST( simulated2DOriented, Dprior )
{
  Pose2 x(1,-9, 0.1);
  Matrix numerical = numericalDerivative11(prior,x);
  Matrix computed;
  prior(x,computed);
  CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
  TEST( simulated2DOriented, DOdo )
{
  Pose2 x1(1,-9,0.1),x2(-5,6,0.2);
  Matrix H1,H2;
  odo(x1,x2,H1,H2);
  Matrix A1 = numericalDerivative21(odo,x1,x2);
  CHECK(assert_equal(A1,H1,1e-9));
  Matrix A2 = numericalDerivative22(odo,x1,x2);
  CHECK(assert_equal(A2,H2,1e-9));
}

/* ************************************************************************* */
TEST( simulated2DOriented, constructor )
{
	Pose2 measurement(0.2, 0.3, 0.1);
	SharedDiagonal model(Vector_(3, 1., 1., 1.));
	Simulated2DOrientedOdometry factor(measurement, model, PoseKey(1), PoseKey(2));

	Config config;
	config.insert(PoseKey(1), Pose2(1., 0., 0.2));
	config.insert(PoseKey(2), Pose2(2., 0., 0.1));
	boost::shared_ptr<GaussianFactor> lf = factor.linearize(config, *config.orderingArbitrary());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
