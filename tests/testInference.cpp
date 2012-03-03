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
TEST( inference, marginals )
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
TEST( inference, marginals2)
{
	planarSLAM::Graph fg;
  SharedDiagonal poseModel(sharedSigma(3, 0.1));
  SharedDiagonal pointModel(sharedSigma(3, 0.1));

  fg.addPrior(0, Pose2(), poseModel);
  fg.addOdometry(0, 1, Pose2(1.0,0.0,0.0), poseModel);
  fg.addOdometry(1, 2, Pose2(1.0,0.0,0.0), poseModel);
  fg.addBearingRange(0, 0, Rot2(), 1.0, pointModel);
  fg.addBearingRange(1, 0, Rot2(), 1.0, pointModel);
  fg.addBearingRange(2, 0, Rot2(), 1.0, pointModel);

  Values init;
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
