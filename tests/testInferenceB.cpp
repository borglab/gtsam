/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInferenceB.cpp
 * @brief   Unit tests for functionality declared in inference.h
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/planarSLAM.h>

using namespace std;
using namespace gtsam;

// Convenience for named keys
Key kx(size_t i) { return Symbol('x',i); }
Key kl(size_t i) { return Symbol('l',i); }

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

  fg.addPrior(kx(0), Pose2(), poseModel);
  fg.addOdometry(kx(0), kx(1), Pose2(1.0,0.0,0.0), poseModel);
  fg.addOdometry(kx(1), kx(2), Pose2(1.0,0.0,0.0), poseModel);
  fg.addBearingRange(kx(0), kl(0), Rot2(), 1.0, pointModel);
  fg.addBearingRange(kx(1), kl(0), Rot2(), 1.0, pointModel);
  fg.addBearingRange(kx(2), kl(0), Rot2(), 1.0, pointModel);

  Values init;
  init.insert(kx(0), Pose2(0.0,0.0,0.0));
  init.insert(kx(1), Pose2(1.0,0.0,0.0));
  init.insert(kx(2), Pose2(2.0,0.0,0.0));
  init.insert(kl(0), Point2(1.0,1.0));

  Ordering ordering(*fg.orderingCOLAMD(init));
  FactorGraph<GaussianFactor>::shared_ptr gfg(fg.linearize(init, ordering));
  GaussianMultifrontalSolver solver(*gfg);
  solver.marginalFactor(ordering[kl(0)]);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
