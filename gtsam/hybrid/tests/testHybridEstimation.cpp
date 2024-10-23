/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridEstimation.cpp
 * @brief   Unit tests for end-to-end Hybrid Estimation
 * @author  Varun Agrawal
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/hybrid/HybridSmoother.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

#include <string>

#include "Switching.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::Z;

namespace estimation_fixture {
std::vector<double> measurements = {0, 1, 2, 2, 2, 2,  3,  4,  5,  6, 6,
                                    7, 8, 9, 9, 9, 10, 11, 11, 11, 11};
// Ground truth discrete seq
std::vector<size_t> discrete_seq = {1, 1, 0, 0, 0, 1, 1, 1, 1, 0,
                                    1, 1, 1, 0, 0, 1, 1, 0, 0, 0};

Switching InitializeEstimationProblem(
    const size_t K, const double between_sigma, const double measurement_sigma,
    const std::vector<double>& measurements,
    const std::string& transitionProbabilityTable,
    HybridNonlinearFactorGraph& graph, Values& initial) {
  Switching switching(K, between_sigma, measurement_sigma, measurements,
                      transitionProbabilityTable);

  // Add prior on M(0)
  graph.push_back(switching.modeChain.at(0));

  // Add the X(0) prior
  graph.push_back(switching.unaryFactors.at(0));
  initial.insert(X(0), switching.linearizationPoint.at<double>(X(0)));

  return switching;
}

}  // namespace estimation_fixture

TEST(HybridEstimation, Full) {
  size_t K = 6;
  std::vector<double> measurements = {0, 1, 2, 2, 2, 3};
  // Ground truth discrete seq
  std::vector<size_t> discrete_seq = {1, 1, 0, 0, 1};
  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  Switching switching(K, 1.0, 0.1, measurements, "1/1 1/1");
  HybridGaussianFactorGraph graph = switching.linearizedFactorGraph;

  Ordering hybridOrdering;
  for (size_t k = 0; k < K; k++) {
    hybridOrdering.push_back(X(k));
  }
  for (size_t k = 0; k < K - 1; k++) {
    hybridOrdering.push_back(M(k));
  }

  HybridBayesNet::shared_ptr bayesNet = graph.eliminateSequential();

  EXPECT_LONGS_EQUAL(2 * K - 1, bayesNet->size());

  HybridValues delta = bayesNet->optimize();

  Values initial = switching.linearizationPoint;
  Values result = initial.retract(delta.continuous());

  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }
  EXPECT(assert_equal(expected_discrete, delta.discrete()));

  Values expected_continuous;
  for (size_t k = 0; k < K; k++) {
    expected_continuous.insert(X(k), measurements[k]);
  }
  EXPECT(assert_equal(expected_continuous, result));
}

/****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST(HybridEstimation, IncrementalSmoother) {
  using namespace estimation_fixture;

  size_t K = 15;

  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  HybridNonlinearFactorGraph graph;
  Values initial;
  Switching switching = InitializeEstimationProblem(K, 1.0, 0.1, measurements,
                                                    "1/1 1/1", graph, initial);
  HybridSmoother smoother;

  HybridGaussianFactorGraph linearized;

  constexpr size_t maxNrLeaves = 3;
  for (size_t k = 1; k < K; k++) {
    if (k > 1) graph.push_back(switching.modeChain.at(k - 1));  // Mode chain
    graph.push_back(switching.binaryFactors.at(k - 1));         // Motion Model
    graph.push_back(switching.unaryFactors.at(k));              // Measurement

    initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    linearized = *graph.linearize(initial);
    Ordering ordering = smoother.getOrdering(linearized);

    smoother.update(linearized, maxNrLeaves, ordering);
    graph.resize(0);
  }

  HybridValues delta = smoother.hybridBayesNet().optimize();

  Values result = initial.retract(delta.continuous());

  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }
  EXPECT(assert_equal(expected_discrete, delta.discrete()));

  Values expected_continuous;
  for (size_t k = 0; k < K; k++) {
    expected_continuous.insert(X(k), measurements[k]);
  }
  EXPECT(assert_equal(expected_continuous, result));
}

/****************************************************************************/
// Test if pruned factor is set to correct error and no errors are thrown.
TEST(HybridEstimation, ValidPruningError) {
  using namespace estimation_fixture;

  size_t K = 8;

  HybridNonlinearFactorGraph graph;
  Values initial;
  Switching switching = InitializeEstimationProblem(K, 1e-2, 1e-3, measurements,
                                                    "1/1 1/1", graph, initial);
  HybridSmoother smoother;

  HybridGaussianFactorGraph linearized;

  constexpr size_t maxNrLeaves = 3;
  for (size_t k = 1; k < K; k++) {
    if (k > 1) graph.push_back(switching.modeChain.at(k - 1));  // Mode chain
    graph.push_back(switching.binaryFactors.at(k - 1));         // Motion Model
    graph.push_back(switching.unaryFactors.at(k));              // Measurement

    initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    linearized = *graph.linearize(initial);
    Ordering ordering = smoother.getOrdering(linearized);

    smoother.update(linearized, maxNrLeaves, ordering);

    graph.resize(0);
  }

  HybridValues delta = smoother.hybridBayesNet().optimize();

  Values result = initial.retract(delta.continuous());

  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }
  EXPECT(assert_equal(expected_discrete, delta.discrete()));

  Values expected_continuous;
  for (size_t k = 0; k < K; k++) {
    expected_continuous.insert(X(k), measurements[k]);
  }
  EXPECT(assert_equal(expected_continuous, result));
}

/****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST(HybridEstimation, ISAM) {
  using namespace estimation_fixture;

  size_t K = 15;

  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  HybridNonlinearFactorGraph graph;
  Values initial;
  Switching switching = InitializeEstimationProblem(K, 1.0, 0.1, measurements,
                                                    "1/1 1/1", graph, initial);
  HybridNonlinearISAM isam;

  HybridGaussianFactorGraph linearized;

  const size_t maxNrLeaves = 3;
  for (size_t k = 1; k < K; k++) {
    if (k > 1) graph.push_back(switching.modeChain.at(k - 1));  // Mode chain
    graph.push_back(switching.binaryFactors.at(k - 1));         // Motion Model
    graph.push_back(switching.unaryFactors.at(k));              // Measurement

    initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    isam.update(graph, initial, maxNrLeaves);
    // isam.saveGraph("NLiSAM" + std::to_string(k) + ".dot");
    // GTSAM_PRINT(isam);

    graph.resize(0);
    initial.clear();
  }

  Values result = isam.estimate();
  DiscreteValues assignment = isam.assignment();

  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }
  EXPECT(assert_equal(expected_discrete, assignment));

  Values expected_continuous;
  for (size_t k = 0; k < K; k++) {
    expected_continuous.insert(X(k), measurements[k]);
  }
  EXPECT(assert_equal(expected_continuous, result));
}

/**
 * @brief Helper method to get the tree of
 * unnormalized probabilities as per the elimination scheme.
 *
 * Used as a helper to compute q(\mu | M, Z) which is used by
 * both P(X | M, Z) and P(M | Z).
 *
 * @param graph The HybridGaussianFactorGraph to eliminate.
 * @return AlgebraicDecisionTree<Key>
 */
AlgebraicDecisionTree<Key> GetProbPrimeTree(
    const HybridGaussianFactorGraph& graph) {
  Ordering continuous(graph.continuousKeySet());
  const auto [bayesNet, remainingGraph] =
      graph.eliminatePartialSequential(continuous);

  auto last_conditional = bayesNet->at(bayesNet->size() - 1);
  DiscreteKeys discrete_keys = last_conditional->discreteKeys();

  const std::vector<DiscreteValues> assignments =
      DiscreteValues::CartesianProduct(discrete_keys);

  std::reverse(discrete_keys.begin(), discrete_keys.end());

  vector<VectorValues::shared_ptr> vector_values;
  for (const DiscreteValues& assignment : assignments) {
    VectorValues values = bayesNet->optimize(assignment);
    vector_values.push_back(std::make_shared<VectorValues>(values));
  }
  DecisionTree<Key, VectorValues::shared_ptr> delta_tree(discrete_keys,
                                                         vector_values);

  // Get the probPrime tree with the correct leaf probabilities
  std::vector<double> probPrimes;
  for (const DiscreteValues& assignment : assignments) {
    VectorValues delta = *delta_tree(assignment);

    // If VectorValues is empty, it means this is a pruned branch.
    // Set the probPrime to 0.0.
    if (delta.size() == 0) {
      probPrimes.push_back(0.0);
      continue;
    }

    double error = graph.error({delta, assignment});
    probPrimes.push_back(exp(-error));
  }
  AlgebraicDecisionTree<Key> probPrimeTree(discrete_keys, probPrimes);
  return probPrimeTree;
}

/*********************************************************************************
 * Test for correctness of different branches of the P'(Continuous | Discrete).
 * The values should match those of P'(Continuous) for each discrete mode.
 ********************************************************************************/
TEST(HybridEstimation, Probability) {
  using namespace estimation_fixture;

  constexpr size_t K = 4;
  double between_sigma = 1.0, measurement_sigma = 0.1;

  // Switching example of robot moving in 1D with
  // given measurements and equal mode priors.
  Switching switching(K, between_sigma, measurement_sigma, measurements,
                      "1/1 1/1");
  auto graph = switching.linearizedFactorGraph;

  // Continuous elimination
  Ordering continuous_ordering(graph.continuousKeySet());
  auto [bayesNet, discreteGraph] =
      graph.eliminatePartialSequential(continuous_ordering);

  // Discrete elimination
  Ordering discrete_ordering(graph.discreteKeySet());
  auto discreteBayesNet = discreteGraph->eliminateSequential(discrete_ordering);

  // Add the discrete conditionals to make it a full bayes net.
  for (auto discrete_conditional : *discreteBayesNet) {
    bayesNet->add(discrete_conditional);
  }

  HybridValues hybrid_values = bayesNet->optimize();

  // This is the correct sequence as designed
  DiscreteValues expectedSequence{{M(0), 1}, {M(1), 1}, {M(2), 0}};
  EXPECT(assert_equal(expectedSequence, hybrid_values.discrete()));
}

/****************************************************************************/
/**
 * Test for correctness of different branches of the P'(Continuous | Discrete)
 * in the multi-frontal setting. The values should match those of P'(Continuous)
 * for each discrete mode.
 */
TEST(HybridEstimation, ProbabilityMultifrontal) {
  using namespace estimation_fixture;

  constexpr size_t K = 4;

  double between_sigma = 1.0, measurement_sigma = 0.1;

  // Switching example of robot moving in 1D with given measurements and equal
  // mode priors.
  Switching switching(K, between_sigma, measurement_sigma, measurements,
                      "1/1 1/1");
  auto graph = switching.linearizedFactorGraph;

  // Get the tree of unnormalized probabilities for each mode sequence.
  AlgebraicDecisionTree<Key> expected_probPrimeTree = GetProbPrimeTree(graph);

  // Eliminate continuous
  Ordering continuous_ordering(graph.continuousKeySet());
  const auto [bayesTree, discreteGraph] =
      graph.eliminatePartialMultifrontal(continuous_ordering);

  // Get the last continuous conditional which will have all the discrete keys
  Key last_continuous_key =
      continuous_ordering.at(continuous_ordering.size() - 1);
  auto last_conditional = (*bayesTree)[last_continuous_key]->conditional();
  DiscreteKeys discrete_keys = last_conditional->discreteKeys();

  Ordering discrete(graph.discreteKeySet());
  auto discreteBayesTree = discreteGraph->eliminateMultifrontal(discrete);

  EXPECT_LONGS_EQUAL(1, discreteBayesTree->size());
  // DiscreteBayesTree should have only 1 clique
  auto discrete_clique = (*discreteBayesTree)[discrete.at(0)];

  std::set<HybridBayesTreeClique::shared_ptr> clique_set;
  for (auto node : bayesTree->nodes()) {
    clique_set.insert(node.second);
  }

  // Set the root of the bayes tree as the discrete clique
  for (auto clique : clique_set) {
    if (clique->conditional()->parents() ==
        discrete_clique->conditional()->frontals()) {
      discreteBayesTree->addClique(clique, discrete_clique);

    } else {
      // Remove the clique from the children of the parents since
      // it will get added again in addClique.
      auto clique_it = std::find(clique->parent()->children.begin(),
                                 clique->parent()->children.end(), clique);
      clique->parent()->children.erase(clique_it);
      discreteBayesTree->addClique(clique, clique->parent());
    }
  }

  HybridValues hybrid_values = discreteBayesTree->optimize();

  // This is the correct sequence as designed
  DiscreteValues expectedSequence{{M(0), 1}, {M(1), 1}, {M(2), 0}};
  EXPECT(assert_equal(expectedSequence, hybrid_values.discrete()));
}

/*********************************************************************************
  // Create a hybrid nonlinear factor graph f(x0, x1, m0; z0, z1)
 ********************************************************************************/
static HybridNonlinearFactorGraph CreateHybridNonlinearFactorGraph() {
  HybridNonlinearFactorGraph nfg;

  constexpr double sigma = 0.5;  // measurement noise
  const auto noise_model = noiseModel::Isotropic::Sigma(1, sigma);

  // Add "measurement" factors:
  nfg.emplace_shared<PriorFactor<double>>(X(0), 0.0, noise_model);
  nfg.emplace_shared<PriorFactor<double>>(X(1), 1.0, noise_model);

  // Add hybrid nonlinear factor:
  DiscreteKey m(M(0), 2);
  const auto zero_motion =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), 0, noise_model);
  const auto one_motion =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), 1, noise_model);
  std::vector<NoiseModelFactor::shared_ptr> components = {zero_motion,
                                                          one_motion};
  nfg.emplace_shared<HybridNonlinearFactor>(m, components);

  return nfg;
}

/*********************************************************************************
  // Create a hybrid linear factor graph f(x0, x1, m0; z0, z1)
 ********************************************************************************/
static HybridGaussianFactorGraph::shared_ptr CreateHybridGaussianFactorGraph() {
  HybridNonlinearFactorGraph nfg = CreateHybridNonlinearFactorGraph();

  Values initial;
  double z0 = 0.0, z1 = 1.0;
  initial.insert<double>(X(0), z0);
  initial.insert<double>(X(1), z1);
  return nfg.linearize(initial);
}

/*********************************************************************************
 * Do hybrid elimination and do regression test on discrete conditional.
 ********************************************************************************/
TEST(HybridEstimation, EliminateSequentialRegression) {
  // Create the factor graph from the nonlinear factor graph.
  HybridGaussianFactorGraph::shared_ptr fg = CreateHybridGaussianFactorGraph();

  // Create expected discrete conditional on m0.
  DiscreteKey m(M(0), 2);
  DiscreteConditional expected(m % "0.51341712/1");  // regression

  // Eliminate into BN using one ordering
  const Ordering ordering1{X(0), X(1), M(0)};
  HybridBayesNet::shared_ptr bn1 = fg->eliminateSequential(ordering1);

  // Check that the discrete conditional matches the expected.
  auto dc1 = bn1->back()->asDiscrete();
  EXPECT(assert_equal(expected, *dc1, 1e-9));

  // Eliminate into BN using a different ordering
  const Ordering ordering2{X(0), X(1), M(0)};
  HybridBayesNet::shared_ptr bn2 = fg->eliminateSequential(ordering2);

  // Check that the discrete conditional matches the expected.
  auto dc2 = bn2->back()->asDiscrete();
  EXPECT(assert_equal(expected, *dc2, 1e-9));
}

/*********************************************************************************
 * Test for correctness via sampling.
 *
 * Compute the conditional P(x0, m0, x1| z0, z1)
 * with measurements z0, z1. To do so, we:
 * 1. Start with the corresponding Factor Graph `FG`.
 * 2. Eliminate the factor graph into a Bayes Net `BN`.
 * 3. Sample from the Bayes Net.
 * 4. Check that the ratio `BN(x)/FG(x) = constant` for all samples `x`.
 ********************************************************************************/
TEST(HybridEstimation, CorrectnessViaSampling) {
  // 1. Create the factor graph from the nonlinear factor graph.
  const auto fg = CreateHybridGaussianFactorGraph();

  // 2. Eliminate into BN
  const HybridBayesNet::shared_ptr bn = fg->eliminateSequential();

  // Set up sampling
  std::mt19937_64 rng(11);

  // Compute the log-ratio between the Bayes net and the factor graph.
  auto compute_ratio = [&](const HybridValues& sample) -> double {
    return bn->evaluate(sample) / fg->probPrime(sample);
  };

  // The error evaluated by the factor graph and the Bayes net should differ by
  // the normalizing term computed via the Bayes net determinant.
  const HybridValues sample = bn->sample(&rng);
  double expected_ratio = compute_ratio(sample);

  // 3. Do sampling
  constexpr int num_samples = 10;
  for (size_t i = 0; i < num_samples; i++) {
    // Sample from the bayes net
    const HybridValues sample = bn->sample(&rng);

    // 4. Check that the ratio is constant.
    EXPECT_DOUBLES_EQUAL(expected_ratio, compute_ratio(sample), 1e-6);
  }
}

/****************************************************************************/
TEST(HybridEstimation, ModeSelection) {
  HybridNonlinearFactorGraph graph;
  Values initial;

  auto measurement_model = noiseModel::Isotropic::Sigma(1, 0.1);
  auto motion_model = noiseModel::Isotropic::Sigma(1, 1.0);

  graph.emplace_shared<PriorFactor<double>>(X(0), 0.0, measurement_model);
  graph.emplace_shared<PriorFactor<double>>(X(1), 0.0, measurement_model);

  // The size of the noise model
  size_t d = 1;
  double noise_tight = 0.5, noise_loose = 5.0;

  auto model0 = std::make_shared<MotionModel>(
           X(0), X(1), 0.0, noiseModel::Isotropic::Sigma(d, noise_loose)),
       model1 = std::make_shared<MotionModel>(
           X(0), X(1), 0.0, noiseModel::Isotropic::Sigma(d, noise_tight));
  std::vector<NoiseModelFactor::shared_ptr> components = {model0, model1};

  HybridNonlinearFactor mf({M(0), 2}, components);

  initial.insert(X(0), 0.0);
  initial.insert(X(1), 0.0);

  auto gmf = mf.linearize(initial);
  graph.add(gmf);

  auto gfg = graph.linearize(initial);

  HybridBayesNet::shared_ptr bayesNet = gfg->eliminateSequential();

  HybridValues delta = bayesNet->optimize();
  EXPECT_LONGS_EQUAL(1, delta.discrete().at(M(0)));

  /**************************************************************/
  HybridBayesNet bn;
  const DiscreteKey mode(M(0), 2);

  bn.push_back(
      GaussianConditional::sharedMeanAndStddev(Z(0), -I_1x1, X(0), Z_1x1, 0.1));
  bn.push_back(
      GaussianConditional::sharedMeanAndStddev(Z(0), -I_1x1, X(1), Z_1x1, 0.1));

  std::vector<std::pair<Vector, double>> parameters{{Z_1x1, noise_loose},
                                                    {Z_1x1, noise_tight}};
  bn.emplace_shared<HybridGaussianConditional>(mode, Z(0), I_1x1, X(0), -I_1x1,
                                               X(1), parameters);

  VectorValues vv;
  vv.insert(Z(0), Z_1x1);

  auto fg = bn.toFactorGraph(vv);
  auto expected_posterior = fg.eliminateSequential();

  EXPECT(assert_equal(*expected_posterior, *bayesNet, 1e-6));
}

/****************************************************************************/
TEST(HybridEstimation, ModeSelection2) {
  using symbol_shorthand::Z;

  // The size of the noise model
  size_t d = 3;
  double noise_tight = 0.5, noise_loose = 5.0;

  HybridBayesNet bn;
  const DiscreteKey mode(M(0), 2);

  bn.push_back(
      GaussianConditional::sharedMeanAndStddev(Z(0), -I_3x3, X(0), Z_3x1, 0.1));
  bn.push_back(
      GaussianConditional::sharedMeanAndStddev(Z(0), -I_3x3, X(1), Z_3x1, 0.1));

  std::vector<std::pair<Vector, double>> parameters{{Z_3x1, noise_loose},
                                                    {Z_3x1, noise_tight}};
  bn.emplace_shared<HybridGaussianConditional>(mode, Z(0), I_3x3, X(0), -I_3x3,
                                               X(1), parameters);

  VectorValues vv;
  vv.insert(Z(0), Z_3x1);

  auto fg = bn.toFactorGraph(vv);

  auto expected_posterior = fg.eliminateSequential();

  // =====================================

  HybridNonlinearFactorGraph graph;
  Values initial;

  auto measurement_model = noiseModel::Isotropic::Sigma(d, 0.1);
  auto motion_model = noiseModel::Isotropic::Sigma(d, 1.0);

  graph.emplace_shared<PriorFactor<Vector3>>(X(0), Z_3x1, measurement_model);
  graph.emplace_shared<PriorFactor<Vector3>>(X(1), Z_3x1, measurement_model);

  auto model0 = std::make_shared<BetweenFactor<Vector3>>(
           X(0), X(1), Z_3x1, noiseModel::Isotropic::Sigma(d, noise_loose)),
       model1 = std::make_shared<BetweenFactor<Vector3>>(
           X(0), X(1), Z_3x1, noiseModel::Isotropic::Sigma(d, noise_tight));
  std::vector<NoiseModelFactor::shared_ptr> components = {model0, model1};

  HybridNonlinearFactor mf({M(0), 2}, components);

  initial.insert<Vector3>(X(0), Z_3x1);
  initial.insert<Vector3>(X(1), Z_3x1);

  auto gmf = mf.linearize(initial);
  graph.add(gmf);

  auto gfg = graph.linearize(initial);

  HybridBayesNet::shared_ptr bayesNet = gfg->eliminateSequential();

  EXPECT(assert_equal(*expected_posterior, *bayesNet, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
