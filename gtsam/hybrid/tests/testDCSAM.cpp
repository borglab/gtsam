/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDCSAM.cpp
 * @brief   Tests for the DCSAM optimization algorithm
 * @author  Kevin Doherty
 * @author  Varun Agrawal
 * @date    March 2025
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/DCSAM.h>
#include <gtsam/hybrid/DiscreteBoundaryFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/inference/BayesNet-inst.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

#include "DiscreteFixture.h"

using namespace gtsam;

using symbol_shorthand::C;
using symbol_shorthand::D;
using symbol_shorthand::L;
using symbol_shorthand::X;

/*
 * Test full DCSAM solve on a mixture factor
 * (as a HybridNonlinearFactor) for 1D case.
 */
TEST(DCSAM, SimpleMixtureFactor) {
  using namespace discrete_mixture_fixture;

  std::vector<NonlinearFactorValuePair> factorComponents{
      {f1(), priorNoise1()->negLogConstant()},
      {fNullHypo(), priorNoiseNullHypo()->negLogConstant()}};

  HybridNonlinearFactor dcMixture(dk, factorComponents);

  // Make an empty hybrid factor graph
  HybridNonlinearFactorGraph hfg;

  hfg.push_back(dcMixture);

  // Let's make an initial guess
  Values initialGuessContinuous;
  double initVal = -2.5;
  initialGuessContinuous.insert(x1, initVal);

  // We also need an initial guess for the discrete variables
  // (this will only be used if it is needed by your factors),
  // here it is ignored.
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 0;

  // Let's make a solver
  DCSAM dcsam;

  HybridValues initialGuess(initialGuessDiscrete, initialGuessContinuous);

  // Add the HybridFactorGraph to DCSAM
  dcsam.update(hfg, initialGuess);

  // Solve
  HybridValues dcvals = dcsam.calculateEstimate();

  // Run another iteration
  dcsam.update();

  // Update HybridValues
  dcvals = dcsam.calculateEstimate();

  // Ensure that the prediction is correct
  const size_t mpeD = dcvals.discrete().at(dk.first);
  EXPECT_LONGS_EQUAL(0, mpeD);
}

namespace slam_fixture {
const double prior_sigma = 0.1;
const double meas_sigma = 1.0;

auto prior_noise = noiseModel::Isotropic::Sigma(3, prior_sigma);
auto meas_noise = noiseModel::Isotropic::Sigma(3, meas_sigma);

Pose2 pose0(0, 0, 0);
Pose2 dx(1, 0, 0.78539816);
Pose2 noise(0.01, 0.01, 0.01);

}  // namespace slam_fixture

/**
 * This is a basic octagonal pose graph SLAM test
 * to verify that DCSAM works on standard SLAM examples.
 */
TEST(DCSAM, SimpleSlamBatch) {
  using namespace slam_fixture;

  // Make a hybrid factor graph
  HybridNonlinearFactorGraph graph;

  // Values for initial guess
  Values initialGuessContinuous;

  Key x0 = X(0);

  PriorFactor<Pose2> p0(x0, pose0, prior_noise);
  graph.push_back(p0);

  initialGuessContinuous.insert(x0, pose0);

  // Setup dcsam
  DCSAM dcsam;

  Pose2 odom(pose0);
  for (size_t i = 0; i < 7; i++) {
    Key xi = X(i);
    Key xj = X(i + 1);

    Pose2 meas = dx * noise;

    BetweenFactor<Pose2> bw(xi, xj, dx * noise, meas_noise);
    graph.push_back(bw);

    odom = odom * meas;
    initialGuessContinuous.insert(xj, odom);
  }

  BetweenFactor<Pose2> bw(x0, X(7), dx * noise, meas_noise);

  HybridValues initialGuess(DiscreteValues(), initialGuessContinuous);
  dcsam.update(graph, initialGuess);
  HybridValues dcvals = dcsam.calculateEstimate();

  VectorValues vvalues;
  for (size_t i = 0; i <= 7; i++) {
    vvalues.insert(X(i), Vector3::Zero());
  }
  // regression check
  EXPECT_DOUBLES_EQUAL(0.0, dcsam.error(vvalues), 1e-8);

  // Values from the DCSAM repo
  Values expectedContinuous;
  expectedContinuous.insert<Pose2>(
      X(0), Pose2(-3.53708e-33, -8.97559e-34, 4.1955e-33));
  expectedContinuous.insert<Pose2>(X(1), Pose2(1, 0.0141421, 0.795398));
  expectedContinuous.insert<Pose2>(X(2), Pose2(1.6899, 0.738184, 1.5908));
  expectedContinuous.insert<Pose2>(X(3), Pose2(1.65576, 1.7377, 2.38619));
  expectedContinuous.insert<Pose2>(X(4), Pose2(0.918069, 2.41298, -3.10159));
  expectedContinuous.insert<Pose2>(X(5), Pose2(-0.0805657, 2.35886, -2.30619));
  expectedContinuous.insert<Pose2>(X(6), Pose2(-0.740961, 1.60781, -1.5108));
  expectedContinuous.insert<Pose2>(X(7), Pose2(-0.66688, 0.61046, -0.715398));

  EXPECT(assert_equal(expectedContinuous, dcvals.nonlinear(), 1e-5));
}

/**
 * This is a basic qualitative octagonal pose graph SLAM test
 * to verify that DCSAM works on standard SLAM examples
 * in the *incremental* setting.
 */
TEST(DCSAM, SimpleSlamIncremental) {
  using namespace slam_fixture;

  // Make a factor graph
  HybridNonlinearFactorGraph graph;

  // Values for initial guess
  Values initialGuessContinuous;

  Key x0 = X(0);

  PriorFactor<Pose2> p0(x0, pose0, prior_noise);

  initialGuessContinuous.insert(x0, pose0);
  graph.push_back(p0);

  // Setup dcsam
  DCSAM dcsam;
  HybridValues initialGuess(DiscreteValues(), initialGuessContinuous);
  dcsam.update(graph, initialGuess);

  graph.resize(0);
  initialGuessContinuous.clear();

  Pose2 odom(pose0);

  for (size_t i = 0; i < 7; i++) {
    Key xi = X(i);
    Key xj = X(i + 1);

    Pose2 meas = dx * noise;

    BetweenFactor<Pose2> bw(xi, xj, dx * noise, meas_noise);
    graph.push_back(bw);

    odom = odom * meas;
    initialGuessContinuous.insert(xj, odom);
    initialGuess =
        HybridValues(initialGuess.discrete(), initialGuessContinuous);
    dcsam.update(graph, initialGuess);

    graph.resize(0);
    initialGuessContinuous.clear();
  }

  BetweenFactor<Pose2> bw(x0, X(7), dx * noise, meas_noise);

  dcsam.update(graph, HybridValues(initialGuess.discrete(), Values()));
  HybridValues dcvals = dcsam.calculateEstimate();

  // Values from the DCSAM repo
  Values expectedContinuous;
  expectedContinuous.insert<Pose2>(
      X(0), Pose2(-3.53708e-33, -8.97559e-34, 4.1955e-33));
  expectedContinuous.insert<Pose2>(X(1), Pose2(1, 0.0141421, 0.795398));
  expectedContinuous.insert<Pose2>(X(2), Pose2(1.6899, 0.738184, 1.5908));
  expectedContinuous.insert<Pose2>(X(3), Pose2(1.65576, 1.7377, 2.38619));
  expectedContinuous.insert<Pose2>(X(4), Pose2(0.918069, 2.41298, -3.10159));
  expectedContinuous.insert<Pose2>(X(5), Pose2(-0.0805657, 2.35886, -2.30619));
  expectedContinuous.insert<Pose2>(X(6), Pose2(-0.740961, 1.60781, -1.5108));
  expectedContinuous.insert<Pose2>(X(7), Pose2(-0.66688, 0.61046, -0.715398));

  EXPECT(assert_equal(expectedContinuous, dcvals.nonlinear(), 1e-5));
}

/**
 * This test is a sanity check that DCSAM works correctly for solely
 discrete problems
 *
 */
TEST(DCSAM, SimpleDiscrete) {
  // Create a DCSAM instance
  DCSAM dcsam;

  // Make an empty hybrid factor graph
  HybridNonlinearFactorGraph hfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(D(1), cardinality);
  const std::vector<double> probs{0.1, 0.9};

  // Make a discrete prior factor and add it to the graph
  DecisionTreeFactor dpf(dk, probs);
  hfg.push_back(dpf);

  // Initial guess for discrete values (only used in certain circumstances)
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 1;

  // Update DCSAM with the new factor
  dcsam.update(hfg, initialGuessDiscrete);

  // Solve
  HybridValues dcvals = dcsam.calculateEstimate();

  // Get the most probable estimate
  const size_t mpeD = dcvals.discrete().at(dk.first);

  // Ensure that the prediction is correct
  EXPECT_LONGS_EQUAL(1, mpeD);
}

/**
 * This is a basic octagonal pose graph SLAM test with two
 * *semantic* landmarks (aka landmarks with unknown measurement
 correspondences)
 * to verify that DCSAM works on standard SLAM examples in
 * the *incremental* setting
 */
TEST(DCSAM, SimpleSemanticSlam) {
  using namespace slam_fixture;

  // Make a factor graph
  HybridNonlinearFactorGraph hfg;

  // Values for initial guess
  Values initialGuessContinuous;
  // Initial guess for discrete values (only used in certain circumstances)
  DiscreteValues initialGuessDiscrete;

  Key x0 = X(0);
  Key l1 = L(1);
  Key lc1 = C(1);
  // Create a discrete key for landmark 1 class with cardinality 2.
  DiscreteKey lm1_class(lc1, 2);

  const double circumradius = (std::sqrt(4 + 2 * std::sqrt(2))) / 2.0;
  Point2 landmark1(circumradius, circumradius);

  auto prior_lm_noise = noiseModel::Isotropic::Sigma(2, prior_sigma);

  // 0.1 rad std on bearing, 10cm on range
  auto br_noise = noiseModel::Isotropic::Sigma(2, 0.1);

  std::vector<double> prior_lm1_class;
  prior_lm1_class.push_back(0.9);
  prior_lm1_class.push_back(0.1);

  PriorFactor<Pose2> p0(x0, pose0, prior_noise);
  // PriorFactor<Point2> pl1(l1, landmark1, prior_lm_noise);

  // add prior on discrete landmark
  DecisionTreeFactor plc1(lm1_class, prior_lm1_class);

  initialGuessContinuous.insert(x0, pose0);
  initialGuessContinuous.insert(l1, landmark1);

  // Set initial guess for discrete class var (ignored internally; only used for
  // MaxMixtures and SumMixtures)
  initialGuessDiscrete[lm1_class.first] = 0;

  hfg.push_back(p0);
  hfg.push_back(plc1);

  // Setup dcsam
  DCSAM dcsam;
  HybridValues initialGuess(initialGuessDiscrete, initialGuessContinuous);
  dcsam.update(hfg, initialGuess);

  HybridValues dcval_start = dcsam.calculateEstimate();

  hfg.resize(0);
  initialGuessContinuous.clear();
  initialGuessDiscrete.clear();

  Pose2 odom(pose0);
  for (size_t i = 0; i < 7; i++) {
    Key xi = X(i);
    Key xj = X(i + 1);

    Pose2 meas = dx * noise;

    BetweenFactor<Pose2> bw(xi, xj, meas, meas_noise);
    hfg.push_back(bw);

    // Add bearing range measurement to landmark in center
    Rot2 bearing1 = Rot2::fromDegrees(67.5);
    const double range1 = circumradius;
    hfg.push_back(
        BearingRangeFactor<Pose2, Point2>(xi, l1, bearing1, range1, br_noise));

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
    DecisionTreeFactor dpf(lm1_class, semantic_meas);
    hfg.push_back(dpf);

    odom = odom * meas;
    initialGuessContinuous.insert(xj, odom);
    initialGuess = HybridValues(DiscreteValues(), initialGuessContinuous);
    dcsam.update(hfg, initialGuess);

    HybridValues dcvals = dcsam.calculateEstimate();

    hfg.resize(0);
    initialGuessContinuous.clear();
  }

  BetweenFactor<Pose2> bw(x0, X(7), dx * noise, meas_noise);

  hfg.push_back(bw);
  dcsam.update(hfg, HybridValues(initialGuess.discrete(), Values()));

  HybridValues dcvals = dcsam.calculateEstimate();

  // Values copied from the DCSAM repo
  DiscreteValues dv;
  dv[lc1] = 1;

  Values nonlinear;
  nonlinear.insert(L(1), Point2(0.501542, 1.20546));
  nonlinear.insert(X(0), Pose2(0.00324255, 0.00959038, -0.0170418));
  nonlinear.insert(X(1), Pose2(1.04653, 0.020688, 0.826187));
  nonlinear.insert(X(2), Pose2(1.7428, 0.798357, 1.6467));
  nonlinear.insert(X(3), Pose2(1.64173, 1.8505, 2.47626));
  nonlinear.insert(X(4), Pose2(0.802545, 2.48895, -2.98168));
  nonlinear.insert(X(5), Pose2(-0.240369, 2.30667, -2.15643));
  nonlinear.insert(X(6), Pose2(-0.819911, 1.4135, -1.3317));
  nonlinear.insert(X(7), Pose2(-0.689416, 0.344402, -0.456848));

  HybridValues expectedValues(VectorValues(), dv, nonlinear);
  EXPECT(assert_equal(expectedValues, dcvals, 1e-5));
}

namespace selection_fixture {
// x0 is pinned at 0, odometry says x1 = x0 + 1 with a tight (mode 0) or loose
// (mode 1) model, and a direct measurement says x1 = 0.
const Key x0 = X(0), x1 = X(1);
const DiscreteKey mode(D(1), 2);
auto tight = noiseModel::Isotropic::Sigma(1, 1.0);
auto loose = noiseModel::Isotropic::Sigma(1, 10.0);

HybridNonlinearFactorGraph Graph() {
  HybridNonlinearFactorGraph graph;
  graph.push_back(PriorFactor<double>(x0, 0.0,
                                      noiseModel::Isotropic::Sigma(1, 1e-3)));
  std::vector<NonlinearFactorValuePair> components{
      {std::make_shared<BetweenFactor<double>>(x0, x1, 1.0, tight),
       tight->negLogConstant()},
      {std::make_shared<BetweenFactor<double>>(x0, x1, 1.0, loose),
       loose->negLogConstant()}};
  graph.push_back(HybridNonlinearFactor(mode, components));
  graph.push_back(PriorFactor<double>(x1, 0.0, tight));
  return graph;
}
}  // namespace selection_fixture

/*
 * Repeated updates must not add the selected component again: the odometry
 * and the measurement have equal weight, so x1 stays at 0.5.
 */
TEST(DCSAM, RepeatedUpdatesKeepOneComponent) {
  using namespace selection_fixture;
  Values initial;
  initial.insert(x0, 0.0);
  initial.insert(x1, 1.0);

  DCSAM dcsam;
  dcsam.update(Graph(),
               HybridValues(VectorValues(), DiscreteValues(), initial));
  for (size_t i = 0; i < 5; i++) {
    HybridValues estimate = dcsam.calculateEstimate();
    EXPECT_LONGS_EQUAL(0, estimate.discrete().at(mode.first));
    EXPECT_DOUBLES_EQUAL(0.5, estimate.nonlinear().at<double>(x1), 1e-6);
    EXPECT_LONGS_EQUAL(3, dcsam.getNonlinearFactorGraph().nrFactors());
    dcsam.update();
  }
}

/*
 * When the discrete estimate changes, the component of the old mode must be
 * replaced, not kept alongside the new one.
 */
TEST(DCSAM, ModeChangeReplacesComponent) {
  using namespace selection_fixture;
  // Far from the measurement, so the first discrete solve picks the loose mode.
  Values initial;
  initial.insert(x0, 0.0);
  initial.insert(x1, 10.0);

  DCSAM dcsam;
  dcsam.update(Graph(),
               HybridValues(VectorValues(), DiscreteValues(), initial));
  // With the loose odometry, x1 = (1/100) / (1/100 + 1).
  EXPECT_DOUBLES_EQUAL(1.0 / 101.0,
                       dcsam.calculateEstimate().nonlinear().at<double>(x1),
                       1e-6);

  // At that estimate the tight mode explains the data better.
  dcsam.update();
  dcsam.update();
  HybridValues estimate = dcsam.calculateEstimate();
  EXPECT_LONGS_EQUAL(0, estimate.discrete().at(mode.first));
  EXPECT_DOUBLES_EQUAL(0.5, estimate.nonlinear().at<double>(x1), 1e-6);
  EXPECT_LONGS_EQUAL(3, dcsam.getNonlinearFactorGraph().nrFactors());
}

/* ************************************************************************* */
namespace slot_reuse_fixture {

// Repeated mode switches reuse slots by default, allowing one spare slot because
// ISAM2 adds the replacement before removing the previous component.
TEST(DCSAM, DefaultReusesSlotsAcrossModeSwitches) {
  using namespace selection_fixture;
  DCSAM dcsam;
  Values initial;
  initial.insert(x0, 0.0);
  initial.insert(x1, 1.0);
  dcsam.update(Graph(), HybridValues(VectorValues(), DiscreteValues(), initial));
  EXPECT_LONGS_EQUAL(3, dcsam.getNonlinearFactorGraph().size());

  for (size_t i = 0; i < 12; ++i) {
    const size_t selectedMode = (i + 1) % 2;
    // Alternate the accumulated discrete odds between 1e-8 and 1e8, so
    // the prior dominates the continuous likelihood in either mode.
    const double small = i == 0 ? 1e-8 : 1e-16;
    HybridNonlinearFactorGraph graph;
    graph.push_back(DecisionTreeFactor(
        mode, selectedMode ? std::vector<double>{small, 1.0}
                           : std::vector<double>{1.0, small}));
    dcsam.update(graph);
    const auto estimate = dcsam.calculateEstimate();
    EXPECT_LONGS_EQUAL(selectedMode, estimate.discrete().at(mode.first));
    EXPECT_DOUBLES_EQUAL(selectedMode ? 1.0 / 101.0 : 0.5,
                         estimate.nonlinear().at<double>(x1), 1e-6);
    EXPECT_LONGS_EQUAL(3, dcsam.getNonlinearFactorGraph().nrFactors());
    EXPECT_LONGS_EQUAL(4, dcsam.getNonlinearFactorGraph().size());
  }
}

// Explicit parameters retain both enabled and disabled slot-reuse behavior,
// and constructing DCSAM does not change the global ISAM2 default.
TEST(DCSAM, ExplicitSlotReuseSettingIsPreserved) {
  using namespace selection_fixture;
  EXPECT(!ISAM2Params().findUnusedFactorSlots);
  for (const bool reuse : {false, true}) {
    ISAM2Params params;
    params.findUnusedFactorSlots = reuse;
    DCSAM dcsam(params);
    Values initial;
    initial.insert(x0, 0.0);
    initial.insert(x1, 1.0);
    dcsam.update(Graph(),
                 HybridValues(VectorValues(), DiscreteValues(), initial));
    for (size_t i = 0; i < 6; ++i) {
      HybridNonlinearFactorGraph graph;
      const double small = i == 0 ? 1e-8 : 1e-16;
      graph.push_back(DecisionTreeFactor(
          mode, i % 2 == 0 ? std::vector<double>{small, 1.0}
                           : std::vector<double>{1.0, small}));
      dcsam.update(graph);
      EXPECT_LONGS_EQUAL(3, dcsam.getNonlinearFactorGraph().nrFactors());
      EXPECT_LONGS_EQUAL(reuse ? 4 : 4 + i,
                        dcsam.getNonlinearFactorGraph().size());
    }
    EXPECT(params.findUnusedFactorSlots == reuse);
  }
  EXPECT(!ISAM2Params().findUnusedFactorSlots);
}

}  // namespace slot_reuse_fixture
/* ************************************************************************* */

/* *************************************************************************
 */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* *************************************************************************
 */
