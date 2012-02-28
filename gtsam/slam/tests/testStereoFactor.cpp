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
 * single camera
 * @author  Chris Beall
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/StereoFactor.h>

#include <gtsam/slam/visualSLAM.h>

using namespace std;
using namespace gtsam;
using namespace boost;
using namespace visualSLAM;

Pose3 camera1(Matrix_(3,3,
		       1., 0., 0.,
		       0.,-1., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,6.25));

Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1500, 1500, 0, 320, 240, 0.5));
StereoCamera stereoCam(Pose3(), K);

// point X Y Z in meters
Point3 p(0, 0, 5);
static SharedNoiseModel sigma(noiseModel::Unit::Create(1));

/* ************************************************************************* */
TEST( StereoFactor, singlePoint)
{
	//Cal3_S2 K(625, 625, 0, 320, 240, 0.5);
	boost::shared_ptr<Cal3_S2Stereo> K(new Cal3_S2Stereo(625, 625, 0, 320, 240, 0.5));
	NonlinearFactorGraph graph;

	graph.add(visualSLAM::PoseConstraint(PoseKey(1),camera1));

	StereoPoint2 z14(320,320.0-50, 240);
  // arguments: measurement, sigma, cam#, measurement #, K, baseline (m)
	graph.add(visualSLAM::StereoFactor(z14,sigma, PoseKey(1), PointKey(1), K));

	// Create a configuration corresponding to the ground truth
	Values values(new Values());
	values.insert(PoseKey(1), camera1); // add camera at z=6.25m looking towards origin

	Point3 l1(0, 0, 0);
	values.insert(PointKey(1), l1);   // add point at origin;

	NonlinearOptimizer::auto_ptr optimizer(new GaussNewtonOptimizer(graph, values)));

	// We expect the initial to be zero because config is the ground truth
	DOUBLES_EQUAL(0.0, optimizer->error(), 1e-9);

	// Iterate once, and the config should not have changed
	NonlinearOptimizer::auto_ptr afterOneIteration = optimizer->iterate();
	DOUBLES_EQUAL(0.0, afterOneIteration->error(), 1e-9);

	// Complete solution
	NonlinearOptimizer::auto_ptr final = optimizer->optimize();

	DOUBLES_EQUAL(0.0, final->error(), 1e-6);
}

/* ************************************************************************* */
	int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
