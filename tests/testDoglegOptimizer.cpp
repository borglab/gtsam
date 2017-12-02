/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DoglegOptimizer.h
 * @brief   Unit tests for DoglegOptimizer
 * @author  Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>

#include <tests/smallExample.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/base/numericalDerivative.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/bind.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <boost/assign/list_of.hpp> // for 'list_of()'
#include <functional>
#include <boost/iterator/counting_iterator.hpp>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST(DoglegOptimizer, ComputeBlend) {
  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      0, Vector2(1.0,2.0), (Matrix(2, 2) << 3.0,4.0,0.0,6.0).finished(),
      3, (Matrix(2, 2) << 7.0,8.0,9.0,10.0).finished(),
      4, (Matrix(2, 2) << 11.0,12.0,13.0,14.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      1, Vector2(15.0,16.0), (Matrix(2, 2) << 17.0,18.0,0.0,20.0).finished(),
      2, (Matrix(2, 2) << 21.0,22.0,23.0,24.0).finished(),
      4, (Matrix(2, 2) << 25.0,26.0,27.0,28.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      2, Vector2(29.0,30.0), (Matrix(2, 2) << 31.0,32.0,0.0,34.0).finished(),
      3, (Matrix(2, 2) << 35.0,36.0,37.0,38.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      3, Vector2(39.0,40.0), (Matrix(2, 2) << 41.0,42.0,0.0,44.0).finished(),
      4, (Matrix(2, 2) << 45.0,46.0,47.0,48.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      4, Vector2(49.0,50.0), (Matrix(2, 2) << 51.0,52.0,0.0,54.0).finished()));

  // Compute steepest descent point
  VectorValues xu = gbn.optimizeGradientSearch();

  // Compute Newton's method point
  VectorValues xn = gbn.optimize();

  // The Newton's method point should be more "adventurous", i.e. larger, than the steepest descent point
  EXPECT(xu.vector().norm() < xn.vector().norm());

  // Compute blend
  double Delta = 1.5;
  VectorValues xb = DoglegOptimizerImpl::ComputeBlend(Delta, xu, xn);
  DOUBLES_EQUAL(Delta, xb.vector().norm(), 1e-10);
}

/* ************************************************************************* */
TEST(DoglegOptimizer, ComputeDoglegPoint) {
  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      0, Vector2(1.0,2.0), (Matrix(2, 2) << 3.0,4.0,0.0,6.0).finished(),
      3, (Matrix(2, 2) << 7.0,8.0,9.0,10.0).finished(),
      4, (Matrix(2, 2) << 11.0,12.0,13.0,14.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      1, Vector2(15.0,16.0), (Matrix(2, 2) << 17.0,18.0,0.0,20.0).finished(),
      2, (Matrix(2, 2) << 21.0,22.0,23.0,24.0).finished(),
      4, (Matrix(2, 2) << 25.0,26.0,27.0,28.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      2, Vector2(29.0,30.0), (Matrix(2, 2) << 31.0,32.0,0.0,34.0).finished(),
      3, (Matrix(2, 2) << 35.0,36.0,37.0,38.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      3, Vector2(39.0,40.0), (Matrix(2, 2) << 41.0,42.0,0.0,44.0).finished(),
      4, (Matrix(2, 2) << 45.0,46.0,47.0,48.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      4, Vector2(49.0,50.0), (Matrix(2, 2) << 51.0,52.0,0.0,54.0).finished()));

  // Compute dogleg point for different deltas

  double Delta1 = 0.5;  // Less than steepest descent
  VectorValues actual1 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta1, gbn.optimizeGradientSearch(), gbn.optimize());
  DOUBLES_EQUAL(Delta1, actual1.vector().norm(), 1e-5);

  double Delta2 = 1.5;  // Between steepest descent and Newton's method
  VectorValues expected2 = DoglegOptimizerImpl::ComputeBlend(Delta2, gbn.optimizeGradientSearch(), gbn.optimize());
  VectorValues actual2 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta2, gbn.optimizeGradientSearch(), gbn.optimize());
  DOUBLES_EQUAL(Delta2, actual2.vector().norm(), 1e-5);
  EXPECT(assert_equal(expected2, actual2));

  double Delta3 = 5.0;  // Larger than Newton's method point
  VectorValues expected3 = gbn.optimize();
  VectorValues actual3 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta3, gbn.optimizeGradientSearch(), gbn.optimize());
  EXPECT(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST(DoglegOptimizer, Iterate) {
  // really non-linear factor graph
  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  // config far from minimum
  Point2 x0(3,0);
  Values config;
  config.insert(X(1), x0);

  double Delta = 1.0;
  for(size_t it=0; it<10; ++it) {
    GaussianBayesNet gbn = *fg.linearize(config)->eliminateSequential();
    // Iterate assumes that linear error = nonlinear error at the linearization point, and this should be true
    double nonlinearError = fg.error(config);
    double linearError = GaussianFactorGraph(gbn).error(config.zeroVectors());
    DOUBLES_EQUAL(nonlinearError, linearError, 1e-5);
//    cout << "it " << it << ", Delta = " << Delta << ", error = " << fg->error(*config) << endl;
    VectorValues dx_u = gbn.optimizeGradientSearch();
    VectorValues dx_n = gbn.optimize();
    DoglegOptimizerImpl::IterationResult result = DoglegOptimizerImpl::Iterate(Delta, DoglegOptimizerImpl::SEARCH_EACH_ITERATION, dx_u, dx_n, gbn, fg, config, fg.error(config));
    Delta = result.delta;
    EXPECT(result.f_error < fg.error(config)); // Check that error decreases
    Values newConfig(config.retract(result.dx_d));
    config = newConfig;
    DOUBLES_EQUAL(fg.error(config), result.f_error, 1e-5); // Check that error is correctly filled in
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
