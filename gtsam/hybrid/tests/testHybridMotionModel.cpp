/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridMotionModel.cpp
 * @brief   Tests hybrid inference with a simple switching motion model
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

#include <memory>

using namespace std;
using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

DiscreteKey m1(M(1), 2);

void addMeasurement(HybridBayesNet &hbn, Key z_key, Key x_key, double sigma) {
  auto measurement_model = noiseModel::Isotropic::Sigma(1, sigma);
  hbn.emplace_shared<GaussianConditional>(z_key, Vector1(0.0), I_1x1, x_key,
                                          -I_1x1, measurement_model);
}

/// Create hybrid motion model p(x1 | x0, m1)
static HybridGaussianConditional::shared_ptr CreateHybridMotionModel(
    double mu0, double mu1, double sigma0, double sigma1) {
  std::vector<std::pair<Vector, double>> motionModels{{Vector1(mu0), sigma0},
                                                      {Vector1(mu1), sigma1}};
  return std::make_shared<HybridGaussianConditional>(m1, X(1), I_1x1, X(0),
                                                     motionModels);
}

/// Create two state Bayes network with 1 or two measurement models
HybridBayesNet CreateBayesNet(
    const HybridGaussianConditional::shared_ptr &hybridMotionModel,
    bool add_second_measurement = false) {
  HybridBayesNet hbn;

  // Add measurement model p(z0 | x0)
  addMeasurement(hbn, Z(0), X(0), 3.0);

  // Optionally add second measurement model p(z1 | x1)
  if (add_second_measurement) {
    addMeasurement(hbn, Z(1), X(1), 3.0);
  }

  // Add hybrid motion model
  hbn.push_back(hybridMotionModel);

  // Discrete uniform prior.
  hbn.emplace_shared<DiscreteConditional>(m1, "50/50");

  return hbn;
}

/// Approximate the discrete marginal P(m1) using importance sampling
std::pair<double, double> approximateDiscreteMarginal(
    const HybridBayesNet &hbn,
    const HybridGaussianConditional::shared_ptr &hybridMotionModel,
    const VectorValues &given, size_t N = 100000) {
  /// Create importance sampling network q(x0,x1,m) = p(x1|x0,m1) q(x0) P(m1),
  /// using q(x0) = N(z0, sigmaQ) to sample x0.
  HybridBayesNet q;
  q.push_back(hybridMotionModel);  // Add hybrid motion model
  q.emplace_shared<GaussianConditional>(GaussianConditional::FromMeanAndStddev(
      X(0), given.at(Z(0)), /* sigmaQ = */ 3.0));  // Add proposal q(x0) for x0
  q.emplace_shared<DiscreteConditional>(m1, "50/50");  // Discrete prior.

  // Do importance sampling
  double w0 = 0.0, w1 = 0.0;
  std::mt19937_64 rng(42);
  for (int i = 0; i < N; i++) {
    HybridValues sample = q.sample(&rng);
    sample.insert(given);
    double weight = hbn.evaluate(sample) / q.evaluate(sample);
    (sample.atDiscrete(M(1)) == 0) ? w0 += weight : w1 += weight;
  }
  double pm1 = w1 / (w0 + w1);
  std::cout << "p(m0) = " << 100 * (1.0 - pm1) << std::endl;
  std::cout << "p(m1) = " << 100 * pm1 << std::endl;
  return {1.0 - pm1, pm1};
}

/* ************************************************************************* */
/**
 * Test a model p(z0|x0)p(z1|x1)p(x1|x0,m1)P(m1).
 *
 * p(x1|x0,m1) has mode-dependent mean but same covariance.
 *
 * Converting to a factor graph gives us ϕ(x0;z0)ϕ(x1;z1)ϕ(x1,x0,m1)P(m1)
 *
 * If we only have a measurement on x0, then
 * the posterior probability of m1 should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(HybridGaussianFactor, TwoStateModel) {
  double mu0 = 1.0, mu1 = 3.0;
  double sigma = 0.5;
  auto hybridMotionModel = CreateHybridMotionModel(mu0, mu1, sigma, sigma);

  // Start with no measurement on x1, only on x0
  const Vector1 z0(0.5);

  VectorValues given;
  given.insert(Z(0), z0);

  {
    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since no measurement on x1, we hedge our bets
    // Importance sampling run with 100k samples gives 50.051/49.949
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "50/50");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete())));
  }

  {
    // If we set z1=4.5 (>> 2.5 which is the halfway point),
    // probability of discrete mode should be leaning to m1==1.
    const Vector1 z1(4.5);
    given.insert(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since we have a measurement on x1, we get a definite result
    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "44.3854/55.6146");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }
}

/* ************************************************************************* */
/**
 * Test a model P(z0|x0)P(x1|x0,m1)P(z1|x1)P(m1).
 *
 * P(x1|x0,m1) has different means and different covariances.
 *
 * Converting to a factor graph gives us
 * ϕ(x0)ϕ(x1,x0,m1)ϕ(x1)P(m1)
 *
 * If we only have a measurement on z0, then
 * the P(m1) should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(HybridGaussianFactor, TwoStateModel2) {
  double mu0 = 1.0, mu1 = 3.0;
  double sigma0 = 0.5, sigma1 = 2.0;
  auto hybridMotionModel = CreateHybridMotionModel(mu0, mu1, sigma0, sigma1);

  // Start with no measurement on x1, only on x0
  const Vector1 z0(0.5);
  VectorValues given;
  given.insert(Z(0), z0);

  {
    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Importance sampling run with 100k samples gives 50.095/49.905
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);

    // Since no measurement on x1, we a 50/50 probability
    auto p_m = bn->at(2)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 0}}), 1e-9);
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 1}}), 1e-9);
  }

  {
    // Now we add a measurement z1 on x1
    const Vector1 z1(4.0);  // favors m==1
    given.insert(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "48.3158/51.6842");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }

  {
    // Add a different measurement z1 on x1 that favors m==0
    const Vector1 z1(1.1);
    given.insert_or_assign(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "55.396/44.604");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }
}

/* ************************************************************************* */
/**
 * Test a model p(z0|x0)p(x1|x0,m1)p(z1|x1)p(m1).
 *
 * p(x1|x0,m1) has the same means but different covariances.
 *
 * Converting to a factor graph gives us
 * ϕ(x0)ϕ(x1,x0,m1)ϕ(x1)p(m1)
 *
 * If we only have a measurement on z0, then
 * the p(m1) should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(HybridGaussianFactor, TwoStateModel3) {
  double mu = 1.0;
  double sigma0 = 0.5, sigma1 = 2.0;
  auto hybridMotionModel = CreateHybridMotionModel(mu, mu, sigma0, sigma1);

  // Start with no measurement on x1, only on x0
  const Vector1 z0(0.5);
  VectorValues given;
  given.insert(Z(0), z0);

  {
    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Importance sampling run with 100k samples gives 50.095/49.905
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);

    // Since no measurement on x1, we a 50/50 probability
    auto p_m = bn->at(2)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 0}}), 1e-9);
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 1}}), 1e-9);
  }

  {
    // Now we add a measurement z1 on x1
    const Vector1 z1(4.0);  // favors m==1
    given.insert(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "51.7762/48.2238");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }

  {
    // Add a different measurement z1 on x1 that favors m==1
    const Vector1 z1(7.0);
    given.insert_or_assign(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "49.0762/50.9238");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.005));
  }
}

/* ************************************************************************* */
/**
 * Same model, P(z0|x0)P(x1|x0,m1)P(z1|x1)P(m1), but now with very informative
 * measurements and vastly different motion model: either stand still or move
 * far. This yields a very informative posterior.
 */
TEST(HybridGaussianFactor, TwoStateModel4) {
  double mu0 = 0.0, mu1 = 10.0;
  double sigma0 = 0.2, sigma1 = 5.0;
  auto hybridMotionModel = CreateHybridMotionModel(mu0, mu1, sigma0, sigma1);

  // We only check the 2-measurement case
  const Vector1 z0(0.0), z1(10.0);
  VectorValues given{{Z(0), z0}, {Z(1), z1}};

  HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
  HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
  HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

  // Values taken from an importance sampling run with 100k samples:
  // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
  DiscreteConditional expected(m1, "8.91527/91.0847");
  EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
