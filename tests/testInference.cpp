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
TEST(GaussianFactorGraph, createSmoother)
{
  using namespace example;
	GaussianFactorGraph fg2;
	Ordering ordering;
	boost::tie(fg2,ordering) = createSmoother(3);
	LONGS_EQUAL(5,fg2.size());

	// eliminate
	vector<Index> x3var; x3var.push_back(ordering["x3"]);
	vector<Index> x1var; x1var.push_back(ordering["x1"]);
	GaussianBayesNet p_x3 = *GaussianSequentialSolver(*GaussianSequentialSolver(fg2).jointFactorGraph(x3var)).eliminate();
	GaussianBayesNet p_x1 = *GaussianSequentialSolver(*GaussianSequentialSolver(fg2).jointFactorGraph(x1var)).eliminate();
	CHECK(assert_equal(*p_x1.back(),*p_x3.front())); // should be the same because of symmetry
}

/* ************************************************************************* */
TEST( Inference, marginals )
{
  using namespace example;
	// create and marginalize a small Bayes net on "x"
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  vector<Index> xvar; xvar.push_back(0);
  GaussianBayesNet actual = *GaussianSequentialSolver(*GaussianSequentialSolver(GaussianFactorGraph(cbn)).jointFactorGraph(xvar)).eliminate();

  // expected is just scalar Gaussian on x
  GaussianBayesNet expected = scalarGaussian(0, 4, sqrt(2));
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Inference, marginals2)
{
  using namespace gtsam::planarSLAM;

  Graph fg;
  SharedDiagonal poseModel(sharedSigma(3, 0.1));
  SharedDiagonal pointModel(sharedSigma(3, 0.1));

  fg.addPrior(PoseKey(0), Pose2(), poseModel);
  fg.addOdometry(PoseKey(0), PoseKey(1), Pose2(1.0,0.0,0.0), poseModel);
  fg.addOdometry(PoseKey(1), PoseKey(2), Pose2(1.0,0.0,0.0), poseModel);
  fg.addBearingRange(PoseKey(0), PointKey(0), Rot2(), 1.0, pointModel);
  fg.addBearingRange(PoseKey(1), PointKey(0), Rot2(), 1.0, pointModel);
  fg.addBearingRange(PoseKey(2), PointKey(0), Rot2(), 1.0, pointModel);

  Values init;
  init.insert(PoseKey(0), Pose2(0.0,0.0,0.0));
  init.insert(PoseKey(1), Pose2(1.0,0.0,0.0));
  init.insert(PoseKey(2), Pose2(2.0,0.0,0.0));
  init.insert(PointKey(0), Point2(1.0,1.0));

  Ordering ordering(*fg.orderingCOLAMD(init));
  FactorGraph<JacobianFactor>::shared_ptr gfg(fg.linearize(init, ordering));
  GaussianMultifrontalSolver solver(*gfg);
  solver.marginalFactor(ordering[PointKey(0)]);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
