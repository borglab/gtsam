/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStereoFactor.cpp
 * @brief   Unit test for StereoFactor
 * @author  Chris Beall
 */

#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static Pose3 camera1(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,6.25));

static boost::shared_ptr<Cal3_S2Stereo> K(new Cal3_S2Stereo(625, 625, 0, 320, 240, 0.5));

// point X Y Z in meters
static Point3 p(0, 0, 5);
static SharedNoiseModel sigma(noiseModel::Unit::Create(1));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( StereoFactor, singlePoint)
{
	NonlinearFactorGraph graph;

	graph.add(NonlinearEquality<Pose3>(X(1), camera1));

	StereoPoint2 z14(320, 320.0-50, 240);
  // arguments: measurement, sigma, cam#, measurement #, K, baseline (m)
	graph.add(GenericStereoFactor<Pose3, Point3>(z14, sigma, X(1), L(1), K));

	// Create a configuration corresponding to the ground truth
	Values values;
	values.insert(X(1), camera1); // add camera at z=6.25m looking towards origin

	Point3 l1(0, 0, 0);
	values.insert(L(1), l1);   // add point at origin;

	GaussNewtonOptimizer optimizer(graph, values);

	// We expect the initial to be zero because config is the ground truth
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

	// Iterate once, and the config should not have changed
	optimizer.iterate();
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

	// Complete solution
	optimizer.optimize();

	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* ************************************************************************* */
	int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
