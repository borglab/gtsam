/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testDCSAM.cpp
 * @brief   Unit tests for DCSAM
 * @author  Kevin Doherty, kdoherty@mit.edu
 * @date    December 2021
 */

// Include for test suite
#include <gtest/gtest.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/BayesNet-inst.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <iomanip>

#ifdef ENABLE_PLOTTING
#include "matplotlibcpp.h"
#endif

// Our custom DCSAM includes
#include "dcsam/DCContinuousFactor.h"
#include "dcsam/DCDiscreteFactor.h"
#include "dcsam/DCEMFactor.h"
#include "dcsam/DCMaxMixtureFactor.h"
#include "dcsam/DCMixtureFactor.h"
#include "dcsam/DCSAM.h"
#include "dcsam/DiscretePriorFactor.h"
#include "dcsam/SemanticBearingRangeFactor.h"
#include "dcsam/SmartDiscretePriorFactor.h"

const double tol = 1e-7;

using namespace std;
using namespace dcsam;

#ifdef ENABLE_PLOTTING
namespace plt = matplotlibcpp;
#endif

/******************************************************************************/

// TEMP just for plotting
// thx @ github gist
template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
  T h = (b - a) / static_cast<T>(N - 1);
  std::vector<T> xs(N);
  typename std::vector<T>::iterator x;
  T val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) *x = val;
  return xs;
}

/*
 * Test simple DiscretePriorFactor. We construct a gtsam::DiscreteFactor factor
 * graph with a single binary variable d1 and a single custom
 * DiscretePriorFactor p(d1) specifying the distribution:
 * p(d1 = 0) = 0.1, p(d1 = 1) = 0.9.
 *
 * After constructing the factor graph, we validate that the marginals are
 * correct (i.e. they match the input p(d1)) and the most probable estimate
 * (computed by maximizing p(d1) over possible values of d1) is 1.
 */
TEST(TestSuite, discrete_prior_factor) {
  // Make an empty discrete factor graph
  gtsam::DiscreteFactorGraph dfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  gtsam::DiscreteKey dk(gtsam::Symbol('d', 1), cardinality);
  const std::vector<double> probs{0.1, 0.9};

  // Make a discrete prior factor and add it to the graph
  DiscretePriorFactor dpf(dk, probs);
  dfg.push_back(dpf);

  // Solve
  gtsam::DiscreteFactor::sharedValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  size_t mpeD = (*mostProbableEstimate).at(dk.first);

  // Get the marginals
  gtsam::DiscreteMarginals discreteMarginals(dfg);
  gtsam::Vector margProbs = discreteMarginals.marginalProbabilities(dk);

  // Verify that each marginal probability is within `tol` of the true marginal
  for (size_t i = 0; i < dk.second; i++) {
    bool margWithinTol = (abs(margProbs[i] - probs[i]) < tol);
    EXPECT_EQ(margWithinTol, true);
  }

  // Ensure that the most probable estimate is correct
  EXPECT_EQ(mpeD, 1);
}

/*
 * Test update-able SmartDiscretePriorFactor/ We construct a
 * gtsam::DiscreteFactor factor graph with a single binary variable d1 and a
 * single custom SmartDiscretePriorFactor p(d1) specifying the distribution:
 * p(d1 = 0) = 0.1, p(d1 = 1) = 0.9.
 *
 * After solving the factor graph, we use the
 * SmartDiscretePriorFactor::updateProbs(newProbs) function to set the factor
 * likelihood to `newProbs` (i.e. modifying the factor likelihood "in place"),
 * giving the distribution: p(d1 = 0) = 0.9, p(d1 = 1) = 0.1.
 *
 * Finally, we re-solve the factor graph and verify that the marginals and
 * the most probable estimate have been correctly updated to reflect `newProbs`
 */
TEST(TestSuite, smart_discrete_prior_factor) {
  // Make an empty discrete factor graph
  gtsam::DiscreteFactorGraph dfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  gtsam::DiscreteKey dk(gtsam::Symbol('d', 1), cardinality);
  const std::vector<double> probs{0.1, 0.9};

  // Make a discrete prior factor and add it to the graph
  SmartDiscretePriorFactor dpf(dk, probs);
  dfg.push_back(dpf);

  // Solve
  gtsam::DiscreteFactor::sharedValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  size_t mpeD = (*mostProbableEstimate).at(dk.first);

  // EXPECT MPE = 1 (exactly the same as above test)
  EXPECT_EQ(mpeD, 1);

  // Get the marginals
  gtsam::DiscreteMarginals discreteMarginals(dfg);
  gtsam::Vector margProbs = discreteMarginals.marginalProbabilities(dk);

  // Update the factor
  const std::vector<double> newProbs{0.9, 0.1};
  boost::shared_ptr<SmartDiscretePriorFactor> smart =
      boost::dynamic_pointer_cast<SmartDiscretePriorFactor>(dfg[0]);
  if (smart) smart->updateProbs(newProbs);

  // Solve
  mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  mpeD = (*mostProbableEstimate).at(dk.first);

  // Get the new marginals
  gtsam::DiscreteMarginals newDiscreteMarginals(dfg);
  gtsam::Vector newMargProbs = newDiscreteMarginals.marginalProbabilities(dk);

  // Verify that each marginal probability is within `tol` of the true marginal
  for (size_t i = 0; i < dk.second; i++) {
    bool margWithinTol = (abs(newMargProbs[i] - newProbs[i]) < tol);
    EXPECT_EQ(margWithinTol, true);
  }

  // Ensure that the prediction is correct
  EXPECT_EQ(mpeD, 0);
}

/*
 * Test DCDiscreteFactor using a simple mixture.
 *
 * Construct a single factor (for a single variable) consisting of a
 * discrete-conditional mixture. Here we have a "null hypothesis" consisting of
 * a Gaussian with large variance and an "alternative hypothesis" consisting of
 * a Gaussian with smaller variance. After initializing the continuous variable
 * far away from the ground-truth solution (x1 = 0), the discrete hypothesis
 * selector will initially choose the null hypothesis.
 */
TEST(TestSuite, dcdiscrete_mixture) {
  // Make an empty discrete factor graph
  DCFactorGraph dcfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  gtsam::DiscreteKey dk(gtsam::Symbol('d', 1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  gtsam::Symbol x1 = gtsam::Symbol('x', 1);
  gtsam::KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis
  const double loc = 0.0;
  const double sigma1 = 1.0;
  gtsam::noiseModel::Isotropic::shared_ptr prior_noise1 =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma1);
  gtsam::PriorFactor<double> f1(x1, loc, prior_noise1);

  // Make a factor for null hypothesis
  const double sigmaNullHypo = 8.0;
  gtsam::noiseModel::Isotropic::shared_ptr prior_noiseNullHypo =
      gtsam::noiseModel::Isotropic::Sigma(1, sigmaNullHypo);

  gtsam::PriorFactor<double> fNullHypo(x1, loc, prior_noiseNullHypo);
  std::vector<gtsam::PriorFactor<double>> factorComponents{f1, fNullHypo};

  DCMixtureFactor<gtsam::PriorFactor<double>> dcMixture(keys, dk,
                                                        factorComponents);
  dcfg.push_back(dcMixture);

  gtsam::DiscreteKey dkTest = dcMixture.discreteKeys()[0];
  std::cout << "DK 1st: " << dkTest.first << std::endl;
  std::cout << "DK 2nd: " << dkTest.second << std::endl;

  // Let's make an initial guess for the continuous values
  gtsam::Values initialGuess;
  double initVal = -2.5;
  initialGuess.insert(x1, initVal);

  // We also need an initial guess for the discrete variables (this will only be
  // used if it is needed by your factors), here it is ignored.
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 0;

  // Let's make a discrete factor graph
  gtsam::DiscreteFactorGraph dfg;

  // Pack DCMixture into a DCDiscreteFactor
  for (auto& it : dcfg) {
    DCDiscreteFactor dcDiscrete(it->discreteKeys()[0], it);
    dfg.push_back(dcDiscrete);
  }

  // Update continuous info
  for (size_t j = 0; j < dfg.size(); j++) {
    boost::shared_ptr<DCDiscreteFactor> dcDiscreteFactor =
        boost::dynamic_pointer_cast<DCDiscreteFactor>(dfg[j]);
    if (dcDiscreteFactor) {
      dcDiscreteFactor->updateContinuous(initialGuess);
      dcDiscreteFactor->updateDiscrete(initialGuessDiscrete);
    }
  }

  // Solve for discrete given continuous
  gtsam::DiscreteFactor::sharedValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  size_t mpeD = (*mostProbableEstimate).at(dk.first);

  // Get the marginals
  gtsam::DiscreteMarginals newDiscreteMarginals(dfg);
  gtsam::Vector newMargProbs = newDiscreteMarginals.marginalProbabilities(dk);

  // Ensure that the prediction is correct
  EXPECT_EQ(mpeD, 1);
}

/*
 * Test DCContinuous using a simple mixture
 *
 * Construct a single factor (for a single variable) consisting of a
 * discrete-conditional mixture. Here we have a "null hypothesis" consisting of
 * a Gaussian with large variance and an "alternative hypothesis" consisting of
 * a Gaussian with smaller variance. After initializing the continuous variable
 * far away from the ground-truth solution (x1 = 0), the discrete hypothesis
 * selector will initially choose the null hypothesis.
 *
 * After a step of continuous optimization, the continuous solution will move to
 * x1 = 0 (since the problem is linear, we have convergence in one step).
 * Finally, updating the discrete value will show that the correct discrete
 * hypothesis will shift to the "alternative hypothesis."
 */
TEST(TestSuite, dccontinuous_mixture) {
  // Make an empty discrete factor graph
  DCFactorGraph dcfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  gtsam::DiscreteKey dk(gtsam::Symbol('d', 1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  gtsam::Symbol x1 = gtsam::Symbol('x', 1);
  gtsam::KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis
  const double loc = 0.0;
  const double sigma1 = 1.0;
  gtsam::noiseModel::Isotropic::shared_ptr prior_noise1 =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma1);
  gtsam::PriorFactor<double> f1(x1, loc, prior_noise1);

  // Make a factor for null hypothesis
  const double sigmaNullHypo = 8.0;
  gtsam::noiseModel::Isotropic::shared_ptr prior_noiseNullHypo =
      gtsam::noiseModel::Isotropic::Sigma(1, sigmaNullHypo);

  gtsam::PriorFactor<double> fNullHypo(x1, loc, prior_noiseNullHypo);
  std::vector<gtsam::PriorFactor<double>> factorComponents{f1, fNullHypo};

  DCMixtureFactor<gtsam::PriorFactor<double>> dcMixture(keys, dk,
                                                        factorComponents);
  dcfg.push_back(dcMixture);

  gtsam::DiscreteKey dkTest = dcMixture.discreteKeys()[0];
  std::cout << "DK 1st: " << dkTest.first << std::endl;
  std::cout << "DK 2nd: " << dkTest.second << std::endl;

// Plot the cost functions for each hypothesis
#ifdef ENABLE_PLOTTING
  // Query cost function
  std::vector<double> xs = linspace(-5.0, 5.0, 50);
  DiscreteValues dv1, dvNH;
  dv1[dk.first] = 0;
  dvNH[dk.first] = 1;
  std::vector<double> errors1;
  std::vector<double> errorsNH;
  gtsam::Values xvals;
  for (size_t i = 0; i < xs.size(); i++) {
    xvals.insert(x1, xs[i]);
    errors1.push_back(dcMixture.error(xvals, dv1));
    errorsNH.push_back(dcMixture.error(xvals, dvNH));
    xvals.clear();
  }

  plt::plot(xs, errors1);
  plt::plot(xs, errorsNH);
#endif

  // Let's make an initial guess for the continuous values
  gtsam::Values initialGuess;
  double initVal = -2.5;
  initialGuess.insert(x1, initVal);

  // And add it to the plot
#ifdef ENABLE_PLOTTING
  std::vector<double> initVec{initVal};
  std::vector<double> initError1{dcMixture.error(initialGuess, dv1)};
  std::vector<double> initErrorNH{dcMixture.error(initialGuess, dvNH)};

  plt::scatter(initVec, initError1, {{"color", "r"}});
  plt::scatter(initVec, initErrorNH, {{"color", "r"}});
#endif

  // We also need an initial guess for the discrete variables (this will only be
  // used if it is needed by your factors), here it is ignored.
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 0;

  // Let's make some factor graphs
  gtsam::DiscreteFactorGraph dfg;
  gtsam::NonlinearFactorGraph graph;

  // Pack DCMixture into a DCContinuousFactor
  for (auto& it : dcfg) {
    DCDiscreteFactor dcDiscrete(it);
    DCContinuousFactor dcContinuous(it);
    dfg.push_back(dcDiscrete);
    graph.push_back(dcContinuous);
  }

  // Update continuous info inside DCDiscreteFactor
  for (size_t j = 0; j < dfg.size(); j++) {
    boost::shared_ptr<DCDiscreteFactor> dcDiscreteFactor =
        boost::dynamic_pointer_cast<DCDiscreteFactor>(dfg[j]);
    if (dcDiscreteFactor) {
      dcDiscreteFactor->updateContinuous(initialGuess);
      dcDiscreteFactor->updateDiscrete(initialGuessDiscrete);
    }
  }

  // Solve for discrete given continuous
  gtsam::DiscreteFactor::sharedValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  size_t mpeD = (*mostProbableEstimate).at(dk.first);

  // From previous test we know this is == 1
  EXPECT_EQ(mpeD, 1);

  // Get the marginals
  gtsam::DiscreteMarginals newDiscreteMarginals(dfg);
  gtsam::Vector newMargProbs = newDiscreteMarginals.marginalProbabilities(dk);

  // Update discrete info inside DCContinuousFactor
  for (size_t j = 0; j < graph.size(); j++) {
    boost::shared_ptr<DCContinuousFactor> dcContinuousFactor =
        boost::dynamic_pointer_cast<DCContinuousFactor>(graph[j]);
    if (dcContinuousFactor)
      dcContinuousFactor->updateDiscrete((*mostProbableEstimate));
  }

  // Setup isam
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  gtsam::ISAM2 isam(isam_params);
  isam.update(graph, initialGuess);

  // Solve for updated continuous value
  gtsam::Values values = isam.calculateEstimate();

  // And add it to the plot
#ifdef ENABLE_PLOTTING
  std::vector<double> updatedVec{values.at<double>(x1)};
  std::vector<double> updatedError1{dcMixture.error(values, dv1)};
  std::vector<double> updatedErrorNH{dcMixture.error(values, dvNH)};

  plt::scatter(updatedVec, updatedError1, {{"color", "b"}});
  plt::scatter(updatedVec, updatedErrorNH, {{"color", "b"}});
  plt::show();
#endif

  // Now update the continuous info in the discrete solver
  for (size_t j = 0; j < dfg.size(); j++) {
    boost::shared_ptr<DCDiscreteFactor> dcDiscreteFactor =
        boost::dynamic_pointer_cast<DCDiscreteFactor>(dfg[j]);
    if (dcDiscreteFactor) dcDiscreteFactor->updateContinuous(values);
    // NOTE: we won't updateDiscrete explicitly here anymore, because we don't
    // need to.
  }

  // Re-solve discrete to verify that output has switched
  mostProbableEstimate = dfg.optimize();
  mpeD = (*mostProbableEstimate).at(dk.first);

  // Ensure that the prediction is correct
  EXPECT_EQ(mpeD, 0);
}

/*
 * Test full DCSAM solve on DCMixtureFactor for 1D case.
 *
 * This is essentially identical to the `DCContinuousFactor` test above, but the
 * solution is obtained by calling the `DCSAM::update` functions (rather than
 * implemented manually as above).
 */
TEST(TestSuite, simple_mixture_factor) {
  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  gtsam::DiscreteKey dk(gtsam::Symbol('d', 1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  gtsam::Symbol x1 = gtsam::Symbol('x', 1);
  gtsam::KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis
  double loc = 0.0;
  double sigma1 = 1.0;
  gtsam::noiseModel::Isotropic::shared_ptr prior_noise1 =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma1);
  gtsam::PriorFactor<double> f1(x1, loc, prior_noise1);

  // Make a factor for null hypothesis
  double sigmaNullHypo = 8.0;
  gtsam::noiseModel::Isotropic::shared_ptr prior_noiseNullHypo =
      gtsam::noiseModel::Isotropic::Sigma(1, sigmaNullHypo);

  gtsam::PriorFactor<double> fNullHypo(x1, loc, prior_noiseNullHypo);
  std::vector<gtsam::PriorFactor<double>> factorComponents{f1, fNullHypo};

  DCMixtureFactor<gtsam::PriorFactor<double>> dcMixture(keys, dk,
                                                        factorComponents);

  // Make an empty hybrid factor graph
  HybridFactorGraph hfg;

  hfg.push_dc(dcMixture);

  gtsam::DiscreteKey dkTest = dcMixture.discreteKeys()[0];
  std::cout << "DK 1st: " << dkTest.first << std::endl;
  std::cout << "DK 2nd: " << dkTest.second << std::endl;

  // Plot the cost functions for each hypothesis
#ifdef ENABLE_PLOTTING
  std::vector<double> xs = linspace(-5.0, 5.0, 50);
  DiscreteValues dv1, dvNH;
  dv1[dk.first] = 0;
  dvNH[dk.first] = 1;
  std::vector<double> errors1;
  std::vector<double> errorsNH;
  gtsam::Values xvals;
  for (size_t i = 0; i < xs.size(); i++) {
    xvals.insert(x1, xs[i]);
    errors1.push_back(dcMixture.error(xvals, dv1));
    errorsNH.push_back(dcMixture.error(xvals, dvNH));
    xvals.clear();
  }

  plt::plot(xs, errors1);
  plt::plot(xs, errorsNH);
#endif

  // Let's make an initial guess
  gtsam::Values initialGuess;
  double initVal = -2.5;
  initialGuess.insert(x1, initVal);

  // And add it to the plot
#ifdef ENABLE_PLOTTING
  std::vector<double> initVec{initVal};
  std::vector<double> initError1{dcMixture.error(initialGuess, dv1)};
  std::vector<double> initErrorNH{dcMixture.error(initialGuess, dvNH)};

  plt::scatter(initVec, initError1, {{"color", "r"}});
  plt::scatter(initVec, initErrorNH, {{"color", "r"}});
#endif

  // We also need an initial guess for the discrete variables (this will only be
  // used if it is needed by your factors), here it is ignored.
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 0;

  // Let's make a solver
  DCSAM dcsam;

  // Add the HybridFactorGraph to DCSAM
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  // Solve
  DCValues dcvals = dcsam.calculateEstimate();

  // Run another iteration
  dcsam.update();

  // Update DCVals
  dcvals = dcsam.calculateEstimate();

  // And add it to the plot
#ifdef ENABLE_PLOTTING
  std::vector<double> updatedVec{dcvals.continuous.at<double>(x1)};
  std::vector<double> updatedError1{dcMixture.error(dcvals.continuous, dv1)};
  std::vector<double> updatedErrorNH{dcMixture.error(dcvals.continuous, dvNH)};

  plt::scatter(updatedVec, updatedError1, {{"color", "b"}});
  plt::scatter(updatedVec, updatedErrorNH, {{"color", "b"}});
  plt::show();
#endif

#ifdef ENABLE_PLOTTING
  plt::show();
#endif

  // Ensure that the prediction is correct
  size_t mpeD = dcvals.discrete.at(dk.first);
  EXPECT_EQ(mpeD, 0);
}

/**
 * This is a basic (qualitative) octagonal pose graph SLAM test to verify that
 * DCSAM works on standard SLAM examples.
 */
TEST(TestSuite, simple_slam_batch) {
  // Make a hybrid factor graph
  HybridFactorGraph graph;

  // Values for initial guess
  gtsam::Values initialGuess;

  gtsam::Symbol x0('x', 0);
  gtsam::Pose2 pose0(0, 0, 0);
  gtsam::Pose2 dx(1, 0, 0.78539816);
  double prior_sigma = 0.1;
  double meas_sigma = 1.0;

  gtsam::noiseModel::Isotropic::shared_ptr prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr meas_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, meas_sigma);

  gtsam::PriorFactor<gtsam::Pose2> p0(x0, pose0, prior_noise);

  initialGuess.insert(x0, pose0);
  graph.push_nonlinear(p0);

  // Setup dcsam
  DCSAM dcsam;

  gtsam::Pose2 odom(pose0);
  gtsam::Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    gtsam::Symbol xi('x', i);
    gtsam::Symbol xj('x', i + 1);

    gtsam::Pose2 meas = dx * noise;

    gtsam::BetweenFactor<gtsam::Pose2> bw(xi, xj, dx * noise, meas_noise);
    graph.push_nonlinear(bw);

    odom = odom * meas;
    initialGuess.insert(xj, odom);
  }

  gtsam::Symbol x7('x', 7);
  gtsam::BetweenFactor<gtsam::Pose2> bw(x0, x7, dx * noise, meas_noise);

  dcsam.update(graph, initialGuess);
  DCValues dcvals = dcsam.calculateEstimate();

  // Plot the robot positions
#ifdef ENABLE_PLOTTING
  std::vector<double> xs, ys;
  for (size_t i = 0; i < 8; i++) {
    xs.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).x());
    ys.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).y());
  }

  plt::plot(xs, ys);
  plt::show();
#endif

  EXPECT_EQ(true, true);
}

/**
 * This is a basic qualitative octagonal pose graph SLAM test to verify that
 * DCSAM works on standard SLAM examples in the *incremental* setting
 */
TEST(TestSuite, simple_slam_incremental) {
  // Make a factor graph
  HybridFactorGraph graph;

  // Values for initial guess
  gtsam::Values initialGuess;

  gtsam::Symbol x0('x', 0);
  gtsam::Pose2 pose0(0, 0, 0);
  gtsam::Pose2 dx(1, 0, 0.78539816);
  double prior_sigma = 0.1;
  double meas_sigma = 1.0;

  gtsam::noiseModel::Isotropic::shared_ptr prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr meas_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, meas_sigma);

  gtsam::PriorFactor<gtsam::Pose2> p0(x0, pose0, prior_noise);

  initialGuess.insert(x0, pose0);
  graph.push_nonlinear(p0);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(graph, initialGuess);

  graph.clear();
  initialGuess.clear();

  gtsam::Pose2 odom(pose0);
  gtsam::Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    gtsam::Symbol xi('x', i);
    gtsam::Symbol xj('x', i + 1);

    gtsam::Pose2 meas = dx * noise;

    gtsam::BetweenFactor<gtsam::Pose2> bw(xi, xj, dx * noise, meas_noise);
    graph.push_nonlinear(bw);

    odom = odom * meas;
    initialGuess.insert(xj, odom);
    dcsam.update(graph, initialGuess);

    graph.clear();
    initialGuess.clear();
  }

  gtsam::Symbol x7('x', 7);
  gtsam::BetweenFactor<gtsam::Pose2> bw(x0, x7, dx * noise, meas_noise);

  dcsam.update(graph, initialGuess);
  DCValues dcvals = dcsam.calculateEstimate();

  // Plot the robot positions
#ifdef ENABLE_PLOTTING
  std::vector<double> xs, ys;
  for (size_t i = 0; i < 8; i++) {
    xs.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).x());
    ys.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).y());
  }
  plt::plot(xs, ys);
  plt::show();
#endif

  EXPECT_EQ(true, true);
}

/**
 * This test is a sanity check that DCSAM works correctly for solely discrete
 * problems
 *
 */
TEST(TestSuite, simple_discrete_dcsam) {
  // Create a DCSAM instance
  DCSAM dcsam;

  // Make an empty hybrid factor graph
  HybridFactorGraph hfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  gtsam::DiscreteKey dk(gtsam::Symbol('d', 1), cardinality);
  const std::vector<double> probs{0.1, 0.9};

  // Make a discrete prior factor and add it to the graph
  DiscretePriorFactor dpf(dk, probs);
  hfg.push_discrete(dpf);

  // Initial guess for discrete values (only used in certain circumstances)
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 1;

  // Update DCSAM with the new factor
  dcsam.update(hfg, initialGuessDiscrete);

  // Solve
  DCValues dcvals = dcsam.calculateEstimate();

  // Get the most probable estimate
  size_t mpeD = dcvals.discrete.at(dk.first);

  // Ensure that the prediction is correct
  EXPECT_EQ(mpeD, 1);
}

/**
 * This is a basic qualitative octagonal pose graph SLAM test with two
 * *semantic* landmarks to verify that DCSAM works on standard SLAM examples in
 * the *incremental* setting
 */
TEST(TestSuite, simple_semantic_slam) {
  // Make a factor graph
  HybridFactorGraph hfg;

  // Values for initial guess
  gtsam::Values initialGuess;
  // Initial guess for discrete values (only used in certain circumstances)
  DiscreteValues initialGuessDiscrete;

  gtsam::Symbol x0('x', 0);
  gtsam::Symbol l1('l', 1);
  gtsam::Symbol lc1('c', 1);
  // Create a discrete key for landmark 1 class with cardinality 2.
  gtsam::DiscreteKey lm1_class(lc1, 2);
  gtsam::Pose2 pose0(0, 0, 0);
  gtsam::Pose2 dx(1, 0, 0.78539816);
  double prior_sigma = 0.1;
  double meas_sigma = 1.0;
  double circumradius = (std::sqrt(4 + 2 * std::sqrt(2))) / 2.0;
  gtsam::Point2 landmark1(circumradius, circumradius);

  gtsam::noiseModel::Isotropic::shared_ptr prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr prior_lm_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr meas_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, meas_sigma);

  // 0.1 rad std on bearing, 10cm on range
  gtsam::noiseModel::Isotropic::shared_ptr br_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

  std::vector<double> prior_lm1_class;
  prior_lm1_class.push_back(0.9);
  prior_lm1_class.push_back(0.1);

  gtsam::PriorFactor<gtsam::Pose2> p0(x0, pose0, prior_noise);
  gtsam::PriorFactor<gtsam::Point2> pl1(l1, landmark1, prior_lm_noise);
  DiscretePriorFactor plc1(lm1_class, prior_lm1_class);

  initialGuess.insert(x0, pose0);
  initialGuess.insert(l1, landmark1);

  // Set initial guess for discrete class var (ignored internally; only used for
  // MaxMixtures and SumMixtures)
  initialGuessDiscrete[lm1_class.first] = 0;

  hfg.push_nonlinear(p0);
  hfg.push_discrete(plc1);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  DCValues dcval_start = dcsam.calculateEstimate();

  hfg.clear();
  initialGuess.clear();
  initialGuessDiscrete.clear();

  gtsam::Pose2 odom(pose0);
  gtsam::Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    gtsam::Symbol xi('x', i);
    gtsam::Symbol xj('x', i + 1);

    gtsam::Pose2 meas = dx * noise;

    gtsam::BetweenFactor<gtsam::Pose2> bw(xi, xj, meas, meas_noise);
    hfg.push_nonlinear(bw);

    // Add bearing range measurement to landmark in center
    gtsam::Rot2 bearing1 = gtsam::Rot2::fromDegrees(67.5);
    double range1 = circumradius;
    hfg.push_nonlinear(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
        xi, l1, bearing1, range1, br_noise));

    // Add semantic measurement to landmark in center
    // For the first couple measurements, pick class=0, later pick class=1
    std::vector<double> semantic_meas;
    if (i < 2) {
      semantic_meas.push_back(0.9);
      semantic_meas.push_back(0.1);
    } else {
      semantic_meas.push_back(0.1);
      semantic_meas.push_back(0.9);
    }
    DiscretePriorFactor dpf(lm1_class, semantic_meas);
    hfg.push_discrete(dpf);

    odom = odom * meas;
    initialGuess.insert(xj, odom);
    dcsam.update(hfg, initialGuess);

    DCValues dcvals = dcsam.calculateEstimate();

    size_t mpeClassL1 = dcvals.discrete.at(lc1);

    // Plot poses and landmarks
#ifdef ENABLE_PLOTTING
    std::vector<double> xs, ys;
    for (size_t j = 0; j < i + 2; j++) {
      xs.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).x());
      ys.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).y());
    }

    std::vector<double> lmxs, lmys;
    lmxs.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
    lmys.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

    string color = (mpeClassL1 == 0) ? "b" : "orange";

    plt::plot(xs, ys);
    plt::scatter(lmxs, lmys, {{"color", color}});
    plt::show();
#endif

    hfg.clear();
    initialGuess.clear();
  }

  gtsam::Symbol x7('x', 7);
  gtsam::BetweenFactor<gtsam::Pose2> bw(x0, x7, dx * noise, meas_noise);

  hfg.push_nonlinear(bw);
  dcsam.update(hfg, initialGuess);

  DCValues dcvals = dcsam.calculateEstimate();

  size_t mpeClassL1 = dcvals.discrete.at(lc1);

  // Plot the poses and landmarks
#ifdef ENABLE_PLOTTING
  std::vector<double> xs, ys;
  for (size_t i = 0; i < 8; i++) {
    xs.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).x());
    ys.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).y());
  }

  std::vector<double> lmxs, lmys;
  lmxs.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
  lmys.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

  string color = (mpeClassL1 == 0) ? "b" : "orange";

  plt::plot(xs, ys);
  plt::scatter(lmxs, lmys, {{"color", color}});
  plt::show();
#endif

  EXPECT_EQ(true, true);
}

/**
 * This unit test verifies that DCFactors added to a DCSAM instance which depend
 * on *previously solved* variables have those variables correctly initialized.
 *
 * The reason we EXPECT_EQ(true, true) is because this test will abort with a
 * failed assertion `allInitialized() == true` in the DCFactor if it does not
 * pass.
 */
TEST(TestSuite, dcsam_initialization) {
  // Make a factor graph.
  HybridFactorGraph hfg;

  // Values for initial guess.
  gtsam::Values initialGuess;
  // Initial guess for discrete variables.
  DiscreteValues initialGuessDiscrete;

  gtsam::Symbol x0('x', 0);
  gtsam::Symbol l1('l', 1);
  gtsam::Symbol lc1('c', 1);

  // Create a discrete key for landmark 1 class with cardinality 2.
  gtsam::DiscreteKey lm1_class(lc1, 2);

  // Set up initial pose
  gtsam::Pose2 pose0(0, 0, 0);

  double prior_sigma = 0.1;
  double meas_sigma = 1.0;
  gtsam::Point2 landmark1(1.0, 1.0);

  /// Noise models for measurements and priors
  // Prior noise for pose and landmarks.
  gtsam::noiseModel::Isotropic::shared_ptr prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr prior_lm_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, prior_sigma);

  // 0.1 rad std on bearing, 10cm on range.
  gtsam::noiseModel::Isotropic::shared_ptr br_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

  // Landmark 1 initial class estimate.
  std::vector<double> prior_lm1_class;
  prior_lm1_class.push_back(0.9);
  prior_lm1_class.push_back(0.1);

  // Add prior factors to the graph and solve.
  gtsam::PriorFactor<gtsam::Pose2> p0(x0, pose0, prior_noise);
  gtsam::PriorFactor<gtsam::Point2> pl1(l1, landmark1, prior_lm_noise);
  DiscretePriorFactor plc1(lm1_class, prior_lm1_class);

  initialGuess.insert(x0, pose0);
  initialGuess.insert(l1, landmark1);
  // Add initial guess for discrete vars (only used for mixtures)
  initialGuessDiscrete[lm1_class.first] = 0;

  hfg.push_nonlinear(p0);
  hfg.push_discrete(plc1);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  DCValues dcval_start = dcsam.calculateEstimate();
  std::cout << "Printing first values" << std::endl;
  dcval_start.discrete.print();

  hfg.clear();
  initialGuess.clear();
  initialGuessDiscrete.clear();

  /**
   * Now DCSAM should have an estimate of the continuous values based on the
   * previous solve. We'll add a new DCFactor now which depends on x0 and l1,
   * but does not explicitly initialize them again when the new factor is added.
   */

  // Add bearing-range measurement from pose (x, y, theta) = (0, 0, 0) to
  // landmark (x, y) = (1, 1)
  gtsam::Rot2 bearing = gtsam::Rot2::fromDegrees(45);
  double range = sqrt(2);
  gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> brfactor(
      x0, l1, bearing, range, br_noise);

  // Set a semantic bearing-range factor up with BR measurement above and
  // semantic measurement equal to the landmark class prior.
  gtsam::KeyVector lm_keys{x0, l1};
  hfg.push_dc(SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
      x0, l1, lm1_class, prior_lm1_class, bearing, range, br_noise));

  // If DCSAM isn't initializing `x0` and `l1` properly for the new factor, this
  // will fail.
  dcsam.update(hfg);

  // Attempt a solve if we made it this far.
  DCValues dcval_final = dcsam.calculateEstimate();

  // If we made it here without an AssertionError, the test passed.
  EXPECT_EQ(true, true);
}

/**
 * This is a basic qualitative octagonal pose graph SLAM test modified to use
 * bearing-range measurements on a *semantic* landmark to verify that DCSAM
 * works on semantic bearing-range measurement SLAM examples in the
 * *incremental* setting
 */
TEST(TestSuite, bearing_range_semantic_slam) {
  // Make a factor graph
  HybridFactorGraph hfg;

  // Values for initial guess
  gtsam::Values initialGuess;
  DiscreteValues initialGuessDiscrete;

  gtsam::Symbol x0('x', 0);
  gtsam::Symbol l1('l', 1);
  gtsam::Symbol lc1('c', 1);
  // Create a discrete key for landmark 1 class with cardinality 2.
  gtsam::DiscreteKey lm1_class(lc1, 2);
  gtsam::Pose2 pose0(0, 0, 0);
  gtsam::Pose2 dx(1, 0, 0.78539816);
  double prior_sigma = 0.1;
  double meas_sigma = 1.0;
  double circumradius = (std::sqrt(4 + 2 * std::sqrt(2))) / 2.0;
  gtsam::Point2 landmark1(circumradius, circumradius);

  gtsam::noiseModel::Isotropic::shared_ptr prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr prior_lm_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr meas_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, meas_sigma);

  // 0.1 rad std on bearing, 10cm on range
  gtsam::noiseModel::Isotropic::shared_ptr br_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

  std::vector<double> prior_lm1_class;
  prior_lm1_class.push_back(0.9);
  prior_lm1_class.push_back(0.1);

  gtsam::PriorFactor<gtsam::Pose2> p0(x0, pose0, prior_noise);
  gtsam::PriorFactor<gtsam::Point2> pl1(l1, landmark1, prior_lm_noise);
  DiscretePriorFactor plc1(lm1_class, prior_lm1_class);

  initialGuess.insert(x0, pose0);
  initialGuess.insert(l1, landmark1);
  initialGuessDiscrete[lm1_class.first] = 0;

  hfg.push_nonlinear(p0);
  hfg.push_nonlinear(pl1);
  hfg.push_discrete(plc1);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  DCValues dcval_start = dcsam.calculateEstimate();
  std::cout << "Printing first values" << std::endl;
  dcval_start.discrete.print();

  hfg.clear();
  initialGuess.clear();
  initialGuessDiscrete.clear();

  gtsam::Pose2 odom(pose0);
  gtsam::Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    gtsam::Symbol xi('x', i);
    gtsam::Symbol xj('x', i + 1);

    gtsam::Pose2 meas = dx * noise;

    gtsam::BetweenFactor<gtsam::Pose2> bw(xi, xj, meas, meas_noise);
    hfg.push_nonlinear(bw);

    // Add semantic bearing-range measurement to landmark in center
    gtsam::Rot2 bearing1 = gtsam::Rot2::fromDegrees(67.5);
    double range1 = circumradius;

    // For the first couple measurements, pick class=0, later pick class=1
    std::vector<double> semantic_meas;
    if (i < 2) {
      semantic_meas.push_back(0.9);
      semantic_meas.push_back(0.1);
    } else {
      semantic_meas.push_back(0.1);
      semantic_meas.push_back(0.9);
    }

    hfg.push_dc(SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
        xi, l1, lm1_class, semantic_meas, bearing1, range1, br_noise));

    odom = odom * meas;
    initialGuess.insert(xj, odom);
    dcsam.update(hfg, initialGuess);

    DCValues dcvals = dcsam.calculateEstimate();

    size_t mpeClassL1 = dcvals.discrete.at(lc1);

    // Plot poses and landmarks
#ifdef ENABLE_PLOTTING
    std::vector<double> xs, ys;
    for (size_t j = 0; j < i + 2; j++) {
      xs.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).x());
      ys.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).y());
    }

    std::vector<double> lmxs, lmys;
    lmxs.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
    lmys.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

    string color = (mpeClassL1 == 0) ? "b" : "orange";

    plt::plot(xs, ys);
    plt::scatter(lmxs, lmys, {{"color", color}});
    plt::show();
#endif

    hfg.clear();
    initialGuess.clear();
  }

  gtsam::Symbol x7('x', 7);
  gtsam::BetweenFactor<gtsam::Pose2> bw(x0, x7, dx * noise, meas_noise);

  hfg.push_nonlinear(bw);
  dcsam.update(hfg, initialGuess);

  DCValues dcvals = dcsam.calculateEstimate();

  size_t mpeClassL1 = dcvals.discrete.at(lc1);

  // Plot the poses and landmarks
#ifdef ENABLE_PLOTTING
  std::vector<double> xs, ys;
  for (size_t i = 0; i < 8; i++) {
    xs.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).x());
    ys.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).y());
  }

  std::vector<double> lmxs, lmys;
  lmxs.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
  lmys.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

  string color = (mpeClassL1 == 0) ? "b" : "orange";

  plt::plot(xs, ys);
  plt::scatter(lmxs, lmys, {{"color", color}});
  plt::show();
#endif

  EXPECT_EQ(mpeClassL1, 1);
}

/**
 * This is a basic qualitative octagonal pose graph SLAM test modified to use a
 * mixture of semantic bearing-range measurements on two *semantic* landmarks
 * to verify that DCSAM works for semantic max-mixtures.
 */
TEST(TestSuite, dcMaxMixture_semantic_slam) {
  // Make a factor graph
  HybridFactorGraph hfg;

  // Values for initial guess
  gtsam::Values initialGuess;
  DiscreteValues initialGuessDiscrete;

  gtsam::Symbol x0('x', 0);
  gtsam::Symbol l1('l', 1);
  gtsam::Symbol lc1('c', 1);
  // Create a discrete key for landmark 1 class with cardinality 2.
  gtsam::DiscreteKey lm1_class(lc1, 2);
  gtsam::Pose2 pose0(0, 0, 0);
  gtsam::Pose2 dx(1, 0, 0.78539816);
  double prior_sigma = 0.1;
  double meas_sigma = 1.0;
  double circumradius = (std::sqrt(4 + 2 * std::sqrt(2))) / 2.0;
  gtsam::Point2 landmark1(circumradius, circumradius);

  gtsam::noiseModel::Isotropic::shared_ptr prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr prior_lm_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr meas_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, meas_sigma);

  // 0.1 rad std on bearing, 10cm on range
  gtsam::noiseModel::Isotropic::shared_ptr br_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

  std::vector<double> prior_lm1_class;
  prior_lm1_class.push_back(0.9);
  prior_lm1_class.push_back(0.1);

  gtsam::PriorFactor<gtsam::Pose2> p0(x0, pose0, prior_noise);
  gtsam::PriorFactor<gtsam::Point2> pl1(l1, landmark1, prior_lm_noise);
  DiscretePriorFactor plc1(lm1_class, prior_lm1_class);

  initialGuess.insert(x0, pose0);
  initialGuess.insert(l1, landmark1);
  initialGuessDiscrete[lm1_class.first] = 0;

  hfg.push_nonlinear(p0);
  hfg.push_nonlinear(pl1);
  hfg.push_discrete(plc1);

  // set up for landmark 2
  gtsam::Symbol l2('l', 2);
  gtsam::Symbol lc2('c', 2);
  // Create a discrete key for landmark 2 class with cardinality 2.
  gtsam::DiscreteKey lm2_class(lc2, 2);
  gtsam::Point2 landmark2(circumradius + .5, circumradius + 5);

  std::vector<double> prior_lm2_class;
  prior_lm2_class.push_back(0.1);
  prior_lm2_class.push_back(0.9);

  gtsam::PriorFactor<gtsam::Point2> pl2(l2, landmark2, prior_lm_noise);
  DiscretePriorFactor plc2(lm2_class, prior_lm2_class);

  initialGuess.insert(l2, landmark2);
  initialGuessDiscrete[lm2_class.first] = 1;

  hfg.push_nonlinear(pl2);
  hfg.push_discrete(plc2);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  DCValues dcval_start = dcsam.calculateEstimate();
  std::cout << "Printing first values" << std::endl;
  dcval_start.discrete.print();

  hfg.clear();
  initialGuess.clear();
  initialGuessDiscrete.clear();

  gtsam::Pose2 odom(pose0);
  gtsam::Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    gtsam::Symbol xi('x', i);
    gtsam::Symbol xj('x', i + 1);

    gtsam::Pose2 meas = dx * noise;

    gtsam::BetweenFactor<gtsam::Pose2> bw(xi, xj, meas, meas_noise);
    hfg.push_nonlinear(bw);

    // Add semantic bearing-range measurement to landmark in center
    gtsam::Rot2 bearing1 = gtsam::Rot2::fromDegrees(67.5);
    double range1 = circumradius;

    // For the first couple measurements, pick class=0, later pick class=1
    std::vector<double> semantic_meas;
    if (i < 2) {
      semantic_meas.push_back(0.9);
      semantic_meas.push_back(0.1);
    } else {
      semantic_meas.push_back(0.1);
      semantic_meas.push_back(0.9);
    }

    gtsam::DiscreteKeys dks({lm1_class, lm2_class});

    // build mixture: dcmaxmixture should be picking the component for lm1
    SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2> sbr1(
        xi, l1, lm1_class, semantic_meas, bearing1, range1, br_noise);
    SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2> sbr2(
        xi, l2, lm2_class, semantic_meas, bearing1, range1, br_noise);
    DCMaxMixtureFactor<SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2>>
        dcmmf({xi, l1, l2}, dks, {sbr1, sbr2}, {.5, .5}, false);

    hfg.push_dc(dcmmf);
    odom = odom * meas;
    initialGuess.insert(xj, odom);
    dcsam.update(hfg, initialGuess);
    DCValues dcvals = dcsam.calculateEstimate();

    size_t mpeClassL1 = dcvals.discrete.at(lc1);

    // Plot poses and landmarks
#ifdef ENABLE_PLOTTING
    std::vector<double> xs, ys;
    for (size_t j = 0; j < i + 2; j++) {
      xs.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).x());
      ys.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).y());
    }

    std::vector<double> lmxs, lmys;
    lmxs.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
    lmys.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

    string color = (mpeClassL1 == 0) ? "b" : "orange";

    plt::plot(xs, ys);
    plt::scatter(lmxs, lmys, {{"color", color}});
    plt::show();
#endif

    hfg.clear();
    initialGuess.clear();
  }

  gtsam::Symbol x7('x', 7);
  gtsam::BetweenFactor<gtsam::Pose2> bw(x0, x7, dx * noise, meas_noise);

  hfg.push_nonlinear(bw);
  dcsam.update(hfg, initialGuess);

  DCValues dcvals = dcsam.calculateEstimate();

  size_t mpeClassL1 = dcvals.discrete.at(lc1);

  // Plot the poses and landmarks
#ifdef ENABLE_PLOTTING
  std::vector<double> xs, ys;
  for (size_t i = 0; i < 8; i++) {
    xs.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).x());
    ys.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).y());
  }

  std::vector<double> lmxs, lmys;
  lmxs.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
  lmys.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

  string color = (mpeClassL1 == 0) ? "b" : "orange";

  plt::plot(xs, ys);
  plt::scatter(lmxs, lmys, {{"color", color}});
  plt::show();
#endif

  EXPECT_EQ(mpeClassL1, 1);
}

TEST(TestSuite, simple_dcemfactor) {
  // Make a factor graph
  HybridFactorGraph hfg;

  // Values for initial guess
  gtsam::Values initialGuess;
  DiscreteValues initialGuessDiscrete;

  gtsam::Symbol x0('x', 0);
  gtsam::Symbol l1('l', 1);
  gtsam::Symbol lc1('c', 1);
  // Create a discrete key for landmark 1 class with cardinality 2.
  gtsam::DiscreteKey lm1_class(lc1, 2);
  gtsam::Pose2 pose0(0, 0, 0);
  gtsam::Pose2 dx(1, 0, 0.78539816);
  double prior_sigma = 0.1;
  double meas_sigma = 1.0;
  double circumradius = (std::sqrt(4 + 2 * std::sqrt(2))) / 2.0;
  gtsam::Point2 landmark1(circumradius, circumradius);

  gtsam::noiseModel::Isotropic::shared_ptr prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr prior_lm_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, prior_sigma);
  gtsam::noiseModel::Isotropic::shared_ptr meas_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, meas_sigma);

  // 0.1 rad std on bearing, 10cm on range
  gtsam::noiseModel::Isotropic::shared_ptr br_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

  std::vector<double> prior_lm1_class;
  prior_lm1_class.push_back(0.9);
  prior_lm1_class.push_back(0.1);

  gtsam::PriorFactor<gtsam::Pose2> p0(x0, pose0, prior_noise);
  gtsam::PriorFactor<gtsam::Point2> pl1(l1, landmark1, prior_lm_noise);
  DiscretePriorFactor plc1(lm1_class, prior_lm1_class);

  initialGuess.insert(x0, pose0);
  initialGuess.insert(l1, landmark1);
  initialGuessDiscrete[lm1_class.first] = 0;

  hfg.push_nonlinear(p0);
  hfg.push_nonlinear(pl1);
  hfg.push_discrete(plc1);

  // set up for landmark 2
  gtsam::Symbol l2('l', 2);
  gtsam::Symbol lc2('c', 2);
  // Create a discrete key for landmark 2 class with cardinality 2.
  gtsam::DiscreteKey lm2_class(lc2, 2);
  gtsam::Point2 landmark2(circumradius + .5, circumradius + 5);

  std::vector<double> prior_lm2_class;
  prior_lm2_class.push_back(0.1);
  prior_lm2_class.push_back(0.9);

  gtsam::PriorFactor<gtsam::Point2> pl2(l2, landmark2, prior_lm_noise);
  DiscretePriorFactor plc2(lm2_class, prior_lm2_class);

  initialGuess.insert(l2, landmark2);
  initialGuessDiscrete[lm2_class.first] = 1;

  hfg.push_nonlinear(pl2);
  hfg.push_discrete(plc2);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  DCValues dcval_start = dcsam.calculateEstimate();
  std::cout << "Printing first values" << std::endl;
  dcval_start.discrete.print();

  hfg.clear();
  initialGuess.clear();
  initialGuessDiscrete.clear();

  gtsam::Pose2 odom(pose0);
  gtsam::Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    gtsam::Symbol xi('x', i);
    gtsam::Symbol xj('x', i + 1);

    gtsam::Pose2 meas = dx * noise;

    gtsam::BetweenFactor<gtsam::Pose2> bw(xi, xj, meas, meas_noise);
    hfg.push_nonlinear(bw);

    // Add semantic bearing-range measurement to landmark in center
    gtsam::Rot2 bearing1 = gtsam::Rot2::fromDegrees(67.5);
    double range1 = circumradius;

    // For the first couple measurements, pick class=0, later pick class=1
    std::vector<double> semantic_meas;
    if (i < 2) {
      semantic_meas.push_back(0.9);
      semantic_meas.push_back(0.1);
    } else {
      semantic_meas.push_back(0.1);
      semantic_meas.push_back(0.9);
    }

    gtsam::DiscreteKeys dks({lm1_class, lm2_class});

    // build mixture: dcemfactor should be picking the component for lm1
    SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2> sbr1(
        xi, l1, lm1_class, semantic_meas, bearing1, range1, br_noise);
    SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2> sbr2(
        xi, l2, lm2_class, semantic_meas, bearing1, range1, br_noise);
    DCEMFactor<SemanticBearingRangeFactor<gtsam::Pose2, gtsam::Point2>> dcemf(
        {xi, l1, l2}, dks, {sbr1, sbr2}, {.5, .5}, false);

    hfg.push_dc(dcemf);
    odom = odom * meas;
    initialGuess.insert(xj, odom);
    dcsam.update(hfg, initialGuess);
    DCValues dcvals = dcsam.calculateEstimate();

    size_t mpeClassL1 = dcvals.discrete.at(lc1);

    // Plot poses and landmarks
#ifdef ENABLE_PLOTTING
    std::vector<double> xs, ys;
    for (size_t j = 0; j < i + 2; j++) {
      xs.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).x());
      ys.push_back(
          dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', j)).y());
    }

    std::vector<double> lmxs, lmys;
    lmxs.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
    lmys.push_back(
        dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

    string color = (mpeClassL1 == 0) ? "b" : "orange";

    plt::plot(xs, ys);
    plt::scatter(lmxs, lmys, {{"color", color}});
    plt::show();
#endif

    hfg.clear();
    initialGuess.clear();
  }

  gtsam::Symbol x7('x', 7);
  gtsam::BetweenFactor<gtsam::Pose2> bw(x0, x7, dx * noise, meas_noise);

  hfg.push_nonlinear(bw);
  dcsam.update(hfg, initialGuess);

  DCValues dcvals = dcsam.calculateEstimate();

  size_t mpeClassL1 = dcvals.discrete.at(lc1);

  // Plot the poses and landmarks
#ifdef ENABLE_PLOTTING
  std::vector<double> xs, ys;
  for (size_t i = 0; i < 8; i++) {
    xs.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).x());
    ys.push_back(dcvals.continuous.at<gtsam::Pose2>(gtsam::Symbol('x', i)).y());
  }

  std::vector<double> lmxs, lmys;
  lmxs.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).x());
  lmys.push_back(
      dcvals.continuous.at<gtsam::Point2>(gtsam::Symbol('l', 1)).y());

  string color = (mpeClassL1 == 0) ? "b" : "orange";

  plt::plot(xs, ys);
  plt::scatter(lmxs, lmys, {{"color", color}});
  plt::show();
#endif

  EXPECT_EQ(mpeClassL1, 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
