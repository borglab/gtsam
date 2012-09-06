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

#include <tests/smallExample.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp> // for 'list_of()'
#include <functional>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
double computeError(const GaussianBayesNet& gbn, const LieVector& values) {

  // Convert Vector to VectorValues
  VectorValues vv = *allocateVectorValues(gbn);
  vv.vector() = values;

  // Convert to factor graph
  GaussianFactorGraph gfg(gbn);
  return gfg.error(vv);
}

/* ************************************************************************* */
double computeErrorBt(const BayesTree<GaussianConditional>& gbt, const LieVector& values) {

  // Convert Vector to VectorValues
  VectorValues vv = *allocateVectorValues(gbt);
  vv.vector() = values;

  // Convert to factor graph
  GaussianFactorGraph gfg(gbt);
  return gfg.error(vv);
}

/* ************************************************************************* */
TEST(DoglegOptimizer, ComputeSteepestDescentPoint) {

  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      0, Vector_(2, 1.0,2.0), Matrix_(2,2, 3.0,4.0,0.0,6.0),
      3, Matrix_(2,2, 7.0,8.0,9.0,10.0),
      4, Matrix_(2,2, 11.0,12.0,13.0,14.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      1, Vector_(2, 15.0,16.0), Matrix_(2,2, 17.0,18.0,0.0,20.0),
      2, Matrix_(2,2, 21.0,22.0,23.0,24.0),
      4, Matrix_(2,2, 25.0,26.0,27.0,28.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      2, Vector_(2, 29.0,30.0), Matrix_(2,2, 31.0,32.0,0.0,34.0),
      3, Matrix_(2,2, 35.0,36.0,37.0,38.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      3, Vector_(2, 39.0,40.0), Matrix_(2,2, 41.0,42.0,0.0,44.0),
      4, Matrix_(2,2, 45.0,46.0,47.0,48.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      4, Vector_(2, 49.0,50.0), Matrix_(2,2, 51.0,52.0,0.0,54.0), ones(2)));

  // Compute the Hessian numerically
  Matrix hessian = numericalHessian(
      boost::function<double(const LieVector&)>(boost::bind(&computeError, gbn, _1)),
      LieVector(VectorValues::Zero(*allocateVectorValues(gbn)).vector()));

  // Compute the gradient numerically
  VectorValues gradientValues = *allocateVectorValues(gbn);
  Vector gradient = numericalGradient(
      boost::function<double(const LieVector&)>(boost::bind(&computeError, gbn, _1)),
      LieVector(VectorValues::Zero(gradientValues).vector()));
  gradientValues.vector() = gradient;

  // Compute the gradient using dense matrices
  Matrix augmentedHessian = GaussianFactorGraph(gbn).augmentedHessian();
  LONGS_EQUAL(11, augmentedHessian.cols());
  VectorValues denseMatrixGradient = *allocateVectorValues(gbn);
  denseMatrixGradient.vector() = -augmentedHessian.col(10).segment(0,10);
  EXPECT(assert_equal(gradientValues, denseMatrixGradient, 1e-5));

  // Compute the steepest descent point
  double step = -gradient.squaredNorm() / (gradient.transpose() * hessian * gradient)(0);
  VectorValues expected = gradientValues;  scal(step, expected);

  // Compute the steepest descent point with the dogleg function
  VectorValues actual = optimizeGradientSearch(gbn);

  // Check that points agree
  EXPECT(assert_equal(expected, actual, 1e-5));

  // Check that point causes a decrease in error
  double origError = GaussianFactorGraph(gbn).error(VectorValues::Zero(*allocateVectorValues(gbn)));
  double newError = GaussianFactorGraph(gbn).error(actual);
  EXPECT(newError < origError);
}

/* ************************************************************************* */
TEST(DoglegOptimizer, BT_BN_equivalency) {

  // Create an arbitrary Bayes Tree
  BayesTree<GaussianConditional> bt;
  bt.insert(BayesTree<GaussianConditional>::sharedClique(new BayesTree<GaussianConditional>::Clique(
      GaussianConditional::shared_ptr(new GaussianConditional(
          boost::assign::pair_list_of
          (2, Matrix_(6,2,
              31.0,32.0,
              0.0,34.0,
              0.0,0.0,
              0.0,0.0,
              0.0,0.0,
              0.0,0.0))
          (3, Matrix_(6,2,
              35.0,36.0,
              37.0,38.0,
              41.0,42.0,
              0.0,44.0,
              0.0,0.0,
              0.0,0.0))
          (4, Matrix_(6,2,
              0.0,0.0,
              0.0,0.0,
              45.0,46.0,
              47.0,48.0,
              51.0,52.0,
              0.0,54.0)),
          3, Vector_(6, 29.0,30.0,39.0,40.0,49.0,50.0), ones(6))))));
  bt.insert(BayesTree<GaussianConditional>::sharedClique(new BayesTree<GaussianConditional>::Clique(
      GaussianConditional::shared_ptr(new GaussianConditional(
          boost::assign::pair_list_of
          (0, Matrix_(4,2,
              3.0,4.0,
              0.0,6.0,
              0.0,0.0,
              0.0,0.0))
          (1, Matrix_(4,2,
              0.0,0.0,
              0.0,0.0,
              17.0,18.0,
              0.0,20.0))
          (2, Matrix_(4,2,
              0.0,0.0,
              0.0,0.0,
              21.0,22.0,
              23.0,24.0))
          (3, Matrix_(4,2,
              7.0,8.0,
              9.0,10.0,
              0.0,0.0,
              0.0,0.0))
          (4, Matrix_(4,2,
              11.0,12.0,
              13.0,14.0,
              25.0,26.0,
              27.0,28.0)),
          2, Vector_(4, 1.0,2.0,15.0,16.0), ones(4))))));

  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      0, Vector_(2, 1.0,2.0), Matrix_(2,2, 3.0,4.0,0.0,6.0),
      3, Matrix_(2,2, 7.0,8.0,9.0,10.0),
      4, Matrix_(2,2, 11.0,12.0,13.0,14.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      1, Vector_(2, 15.0,16.0), Matrix_(2,2, 17.0,18.0,0.0,20.0),
      2, Matrix_(2,2, 21.0,22.0,23.0,24.0),
      4, Matrix_(2,2, 25.0,26.0,27.0,28.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      2, Vector_(2, 29.0,30.0), Matrix_(2,2, 31.0,32.0,0.0,34.0),
      3, Matrix_(2,2, 35.0,36.0,37.0,38.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      3, Vector_(2, 39.0,40.0), Matrix_(2,2, 41.0,42.0,0.0,44.0),
      4, Matrix_(2,2, 45.0,46.0,47.0,48.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      4, Vector_(2, 49.0,50.0), Matrix_(2,2, 51.0,52.0,0.0,54.0), ones(2)));

  GaussianFactorGraph expected(gbn);
  GaussianFactorGraph actual(bt);

  EXPECT(assert_equal(expected.augmentedHessian(), actual.augmentedHessian()));
}

/* ************************************************************************* */
TEST(DoglegOptimizer, ComputeSteepestDescentPointBT) {

  // Create an arbitrary Bayes Tree
  BayesTree<GaussianConditional> bt;
  bt.insert(BayesTree<GaussianConditional>::sharedClique(new BayesTree<GaussianConditional>::Clique(
      GaussianConditional::shared_ptr(new GaussianConditional(
          boost::assign::pair_list_of
          (2, Matrix_(6,2,
              31.0,32.0,
              0.0,34.0,
              0.0,0.0,
              0.0,0.0,
              0.0,0.0,
              0.0,0.0))
          (3, Matrix_(6,2,
              35.0,36.0,
              37.0,38.0,
              41.0,42.0,
              0.0,44.0,
              0.0,0.0,
              0.0,0.0))
          (4, Matrix_(6,2,
              0.0,0.0,
              0.0,0.0,
              45.0,46.0,
              47.0,48.0,
              51.0,52.0,
              0.0,54.0)),
          3, Vector_(6, 29.0,30.0,39.0,40.0,49.0,50.0), ones(6))))));
  bt.insert(BayesTree<GaussianConditional>::sharedClique(new BayesTree<GaussianConditional>::Clique(
      GaussianConditional::shared_ptr(new GaussianConditional(
          boost::assign::pair_list_of
          (0, Matrix_(4,2,
              3.0,4.0,
              0.0,6.0,
              0.0,0.0,
              0.0,0.0))
          (1, Matrix_(4,2,
              0.0,0.0,
              0.0,0.0,
              17.0,18.0,
              0.0,20.0))
          (2, Matrix_(4,2,
              0.0,0.0,
              0.0,0.0,
              21.0,22.0,
              23.0,24.0))
          (3, Matrix_(4,2,
              7.0,8.0,
              9.0,10.0,
              0.0,0.0,
              0.0,0.0))
          (4, Matrix_(4,2,
              11.0,12.0,
              13.0,14.0,
              25.0,26.0,
              27.0,28.0)),
          2, Vector_(4, 1.0,2.0,15.0,16.0), ones(4))))));

  // Compute the Hessian numerically
  Matrix hessian = numericalHessian(
      boost::function<double(const LieVector&)>(boost::bind(&computeErrorBt, bt, _1)),
      LieVector(VectorValues::Zero(*allocateVectorValues(bt)).vector()));

  // Compute the gradient numerically
  VectorValues gradientValues = *allocateVectorValues(bt);
  Vector gradient = numericalGradient(
      boost::function<double(const LieVector&)>(boost::bind(&computeErrorBt, bt, _1)),
      LieVector(VectorValues::Zero(gradientValues).vector()));
  gradientValues.vector() = gradient;

  // Compute the gradient using dense matrices
  Matrix augmentedHessian = GaussianFactorGraph(bt).augmentedHessian();
  LONGS_EQUAL(11, augmentedHessian.cols());
  VectorValues denseMatrixGradient = *allocateVectorValues(bt);
  denseMatrixGradient.vector() = -augmentedHessian.col(10).segment(0,10);
  EXPECT(assert_equal(gradientValues, denseMatrixGradient, 1e-5));

  // Compute the steepest descent point
  double step = -gradient.squaredNorm() / (gradient.transpose() * hessian * gradient)(0);
  VectorValues expected = gradientValues;  scal(step, expected);

  // Known steepest descent point from Bayes' net version
  VectorValues expectedFromBN(5,2);
  expectedFromBN[0] = Vector_(2, 0.000129034, 0.000688183);
  expectedFromBN[1] = Vector_(2, 0.0109679, 0.0253767);
  expectedFromBN[2] = Vector_(2, 0.0680441, 0.114496);
  expectedFromBN[3] = Vector_(2, 0.16125, 0.241294);
  expectedFromBN[4] = Vector_(2, 0.300134, 0.423233);

  // Compute the steepest descent point with the dogleg function
  VectorValues actual = optimizeGradientSearch(bt);

  // Check that points agree
  EXPECT(assert_equal(expected, actual, 1e-5));
  EXPECT(assert_equal(expectedFromBN, actual, 1e-5));

  // Check that point causes a decrease in error
  double origError = GaussianFactorGraph(bt).error(VectorValues::Zero(*allocateVectorValues(bt)));
  double newError = GaussianFactorGraph(bt).error(actual);
  EXPECT(newError < origError);
}

/* ************************************************************************* */
TEST(DoglegOptimizer, ComputeBlend) {
  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      0, Vector_(2, 1.0,2.0), Matrix_(2,2, 3.0,4.0,0.0,6.0),
      3, Matrix_(2,2, 7.0,8.0,9.0,10.0),
      4, Matrix_(2,2, 11.0,12.0,13.0,14.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      1, Vector_(2, 15.0,16.0), Matrix_(2,2, 17.0,18.0,0.0,20.0),
      2, Matrix_(2,2, 21.0,22.0,23.0,24.0),
      4, Matrix_(2,2, 25.0,26.0,27.0,28.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      2, Vector_(2, 29.0,30.0), Matrix_(2,2, 31.0,32.0,0.0,34.0),
      3, Matrix_(2,2, 35.0,36.0,37.0,38.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      3, Vector_(2, 39.0,40.0), Matrix_(2,2, 41.0,42.0,0.0,44.0),
      4, Matrix_(2,2, 45.0,46.0,47.0,48.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      4, Vector_(2, 49.0,50.0), Matrix_(2,2, 51.0,52.0,0.0,54.0), ones(2)));

  // Compute steepest descent point
  VectorValues xu = optimizeGradientSearch(gbn);

  // Compute Newton's method point
  VectorValues xn = optimize(gbn);

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
      0, Vector_(2, 1.0,2.0), Matrix_(2,2, 3.0,4.0,0.0,6.0),
      3, Matrix_(2,2, 7.0,8.0,9.0,10.0),
      4, Matrix_(2,2, 11.0,12.0,13.0,14.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      1, Vector_(2, 15.0,16.0), Matrix_(2,2, 17.0,18.0,0.0,20.0),
      2, Matrix_(2,2, 21.0,22.0,23.0,24.0),
      4, Matrix_(2,2, 25.0,26.0,27.0,28.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      2, Vector_(2, 29.0,30.0), Matrix_(2,2, 31.0,32.0,0.0,34.0),
      3, Matrix_(2,2, 35.0,36.0,37.0,38.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      3, Vector_(2, 39.0,40.0), Matrix_(2,2, 41.0,42.0,0.0,44.0),
      4, Matrix_(2,2, 45.0,46.0,47.0,48.0), ones(2)));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      4, Vector_(2, 49.0,50.0), Matrix_(2,2, 51.0,52.0,0.0,54.0), ones(2)));

  // Compute dogleg point for different deltas

  double Delta1 = 0.5;  // Less than steepest descent
  VectorValues actual1 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta1, optimizeGradientSearch(gbn), optimize(gbn));
  DOUBLES_EQUAL(Delta1, actual1.vector().norm(), 1e-5);

  double Delta2 = 1.5;  // Between steepest descent and Newton's method
  VectorValues expected2 = DoglegOptimizerImpl::ComputeBlend(Delta2, optimizeGradientSearch(gbn), optimize(gbn));
  VectorValues actual2 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta2, optimizeGradientSearch(gbn), optimize(gbn));
  DOUBLES_EQUAL(Delta2, actual2.vector().norm(), 1e-5);
  EXPECT(assert_equal(expected2, actual2));

  double Delta3 = 5.0;  // Larger than Newton's method point
  VectorValues expected3 = optimize(gbn);
  VectorValues actual3 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta3, optimizeGradientSearch(gbn), optimize(gbn));
  EXPECT(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST(DoglegOptimizer, Iterate) {
  // really non-linear factor graph
  boost::shared_ptr<example::Graph> fg(new example::Graph(
      example::createReallyNonlinearFactorGraph()));

  // config far from minimum
  Point2 x0(3,0);
  boost::shared_ptr<Values> config(new Values);
  config->insert(X(1), x0);

  // ordering
  boost::shared_ptr<Ordering> ord(new Ordering());
  ord->push_back(X(1));

  double Delta = 1.0;
  for(size_t it=0; it<10; ++it) {
    GaussianSequentialSolver solver(*fg->linearize(*config, *ord));
    GaussianBayesNet gbn = *solver.eliminate();
    // Iterate assumes that linear error = nonlinear error at the linearization point, and this should be true
    double nonlinearError = fg->error(*config);
    double linearError = GaussianFactorGraph(gbn).error(VectorValues::Zero(*allocateVectorValues(gbn)));
    DOUBLES_EQUAL(nonlinearError, linearError, 1e-5);
//    cout << "it " << it << ", Delta = " << Delta << ", error = " << fg->error(*config) << endl;
    DoglegOptimizerImpl::IterationResult result = DoglegOptimizerImpl::Iterate(Delta, DoglegOptimizerImpl::SEARCH_EACH_ITERATION, gbn, *fg, *config, *ord, fg->error(*config));
    Delta = result.Delta;
    EXPECT(result.f_error < fg->error(*config)); // Check that error decreases
    Values newConfig(config->retract(result.dx_d, *ord));
    (*config) = newConfig;
    DOUBLES_EQUAL(fg->error(*config), result.f_error, 1e-5); // Check that error is correctly filled in
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
