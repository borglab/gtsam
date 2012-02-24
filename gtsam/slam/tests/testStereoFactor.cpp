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
#include <gtsam/nonlinear/NonlinearOptimizer.h>
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
	boost::shared_ptr<visualSLAM::Graph> graph(new visualSLAM::Graph());

	graph->add(visualSLAM::PoseConstraint(PoseKey(1),camera1));

	StereoPoint2 z14(320,320.0-50, 240);
  // arguments: measurement, sigma, cam#, measurement #, K, baseline (m)
	graph->add(visualSLAM::StereoFactor(z14,sigma, PoseKey(1), PointKey(1), K));

	// Create a configuration corresponding to the ground truth
	boost::shared_ptr<Values> values(new Values());
	values->insert(PoseKey(1), camera1); // add camera at z=6.25m looking towards origin

	Point3 l1(0, 0, 0);
	values->insert(PointKey(1), l1);   // add point at origin;

	Ordering::shared_ptr ordering = graph->orderingCOLAMD(*values);

	typedef gtsam::NonlinearOptimizer<visualSLAM::Graph, gtsam::GaussianFactorGraph, gtsam::GaussianMultifrontalSolver> Optimizer;   // optimization engine for this domain

	double absoluteThreshold = 1e-9;
	double relativeThreshold = 1e-5;
	int maxIterations = 100;
	NonlinearOptimizationParameters::verbosityLevel verbose = NonlinearOptimizationParameters::SILENT;
	NonlinearOptimizationParameters parameters(absoluteThreshold, relativeThreshold, 0,
			maxIterations, 1.0, 10, verbose, NonlinearOptimizationParameters::BOUNDED);

	Optimizer optimizer(graph, values, ordering, make_shared<NonlinearOptimizationParameters>(parameters));

	// We expect the initial to be zero because config is the ground truth
	DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

	// Iterate once, and the config should not have changed
	Optimizer afterOneIteration = optimizer.iterate();
	DOUBLES_EQUAL(0.0, afterOneIteration.error(), 1e-9);

	// Complete solution
	Optimizer final = optimizer.gaussNewton();

	DOUBLES_EQUAL(0.0, final.error(), 1e-6);
}

/* ************************************************************************* */
	int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
