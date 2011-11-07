/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInference.cpp
 * @brief   Unit tests for functionality declared in inference.h
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/planarSLAM.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// The tests below test the *generic* inference algorithms. Some of these have
// specialized versions in the derived classes GaussianFactorGraph etc...
/* ************************************************************************* */

/* ************************************************************************* */
TEST( Inference, marginals )
{
  using namespace example;
	// create and marginalize a small Bayes net on "x"
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  vector<Index> xvar; xvar.push_back(0);
  GaussianBayesNet actual = *GaussianSequentialSolver(
  		*GaussianSequentialSolver(GaussianFactorGraph(cbn)).jointFactorGraph(xvar)).eliminate();

  // expected is just scalar Gaussian on x
  GaussianBayesNet expected = scalarGaussian(0, 4, sqrt(2));
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Inference, marginals2)
{
	planarSLAM::Graph fg;
  SharedDiagonal poseModel(sharedSigma(3, 0.1));
  SharedDiagonal pointModel(sharedSigma(3, 0.1));

  fg.addPrior(planarSLAM::PoseKey(0), Pose2(), poseModel);
  fg.addOdometry(planarSLAM::PoseKey(0), planarSLAM::PoseKey(1), Pose2(1.0,0.0,0.0), poseModel);
  fg.addOdometry(planarSLAM::PoseKey(1), planarSLAM::PoseKey(2), Pose2(1.0,0.0,0.0), poseModel);
  fg.addBearingRange(planarSLAM::PoseKey(0), planarSLAM::PointKey(0), Rot2(), 1.0, pointModel);
  fg.addBearingRange(planarSLAM::PoseKey(1), planarSLAM::PointKey(0), Rot2(), 1.0, pointModel);
  fg.addBearingRange(planarSLAM::PoseKey(2), planarSLAM::PointKey(0), Rot2(), 1.0, pointModel);

  planarSLAM::Values init;
  init.insert(planarSLAM::PoseKey(0), Pose2(0.0,0.0,0.0));
  init.insert(planarSLAM::PoseKey(1), Pose2(1.0,0.0,0.0));
  init.insert(planarSLAM::PoseKey(2), Pose2(2.0,0.0,0.0));
  init.insert(planarSLAM::PointKey(0), Point2(1.0,1.0));

  Ordering ordering(*fg.orderingCOLAMD(init));
  FactorGraph<GaussianFactor>::shared_ptr gfg(fg.linearize(init, ordering));
  GaussianMultifrontalSolver solver(*gfg);
  solver.marginalFactor(ordering[planarSLAM::PointKey(0)]);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
