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

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

#include <tests/smallExample.h>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

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
  GaussianBayesNet expected = scalarGaussian(0, 4, sqrt(2.0));
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( inference, marginals2)
{
	NonlinearFactorGraph fg;
  SharedDiagonal poseModel(noiseModel::Isotropic::Sigma(3, 0.1));
  SharedDiagonal pointModel(noiseModel::Isotropic::Sigma(3, 0.1));

  fg.add(PriorFactor<Pose2>(X(0), Pose2(), poseModel));
  fg.add(BetweenFactor<Pose2>(X(0), X(1), Pose2(1.0,0.0,0.0), poseModel));
  fg.add(BetweenFactor<Pose2>(X(1), X(2), Pose2(1.0,0.0,0.0), poseModel));
  fg.add(BearingRangeFactor<Pose2, Point2>(X(0), L(0), Rot2(), 1.0, pointModel));
  fg.add(BearingRangeFactor<Pose2, Point2>(X(1), L(0), Rot2(), 1.0, pointModel));
  fg.add(BearingRangeFactor<Pose2, Point2>(X(2), L(0), Rot2(), 1.0, pointModel));

  Values init;
  init.insert(X(0), Pose2(0.0,0.0,0.0));
  init.insert(X(1), Pose2(1.0,0.0,0.0));
  init.insert(X(2), Pose2(2.0,0.0,0.0));
  init.insert(L(0), Point2(1.0,1.0));

  Ordering ordering(*fg.orderingCOLAMD(init));
  FactorGraph<GaussianFactor>::shared_ptr gfg(fg.linearize(init, ordering));
  GaussianMultifrontalSolver solver(*gfg);
  solver.marginalFactor(ordering[L(0)]);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
