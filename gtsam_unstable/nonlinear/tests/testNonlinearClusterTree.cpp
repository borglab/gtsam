/**
 * @file testNonlinearClusterTree.cpp
 * @author Frank Dellaert
 * @date   March, 2016
 */

#include <gtsam_unstable/nonlinear/NonlinearClusterTree.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

static const Symbol x1('x', 1), x2('x', 2), x3('x', 3);
static const Symbol l1('l', 1), l2('l', 2);

/* ************************************************************************* */
NonlinearFactorGraph planarSLAMGraph() {
  NonlinearFactorGraph graph;

  // Prior on pose x1 at the origin.
  Pose2 prior(0.0, 0.0, 0.0);
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(x1, prior, priorNoise);

  // Two odometry factors
  Pose2 odometry(2.0, 0.0, 0.0);
  auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2> >(x1, x2, odometry, odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(x2, x3, odometry, odometryNoise);

  // Add Range-Bearing measurements to two different landmarks
  auto measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2));
  Rot2 bearing11 = Rot2::fromDegrees(45), bearing21 = Rot2::fromDegrees(90),
       bearing32 = Rot2::fromDegrees(90);
  double range11 = std::sqrt(4.0 + 4.0), range21 = 2.0, range32 = 2.0;
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x1, l1, bearing11, range11, measurementNoise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x2, l1, bearing21, range21, measurementNoise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x3, l2, bearing32, range32, measurementNoise);

  return graph;
}

/* ************************************************************************* */
// Create initial estimate
Values planarSLAMValues() {
  Values initial;
  initial.insert(l1, Point2(1.8, 2.1));
  initial.insert(l2, Point2(4.1, 1.8));
  initial.insert(x1, Pose2(0.5, 0.0, 0.2));
  initial.insert(x2, Pose2(2.3, 0.1, -0.2));
  initial.insert(x3, Pose2(4.1, 0.1, 0.1));
  return initial;
}

/* ************************************************************************* */
TEST(NonlinearClusterTree, Clusters) {
  NonlinearFactorGraph graph = planarSLAMGraph();
  Values initial = planarSLAMValues();

  // Build the clusters
  // NOTE(frank): Order matters here as factors are removed!
  VariableIndex variableIndex(graph);
  typedef NonlinearClusterTree::NonlinearCluster Cluster;
  auto marginalCluster = std::shared_ptr<Cluster>(new Cluster(variableIndex, {x1}, &graph));
  auto landmarkCluster = std::shared_ptr<Cluster>(new Cluster(variableIndex, {l1, l2}, &graph));
  auto rootCluster = std::shared_ptr<Cluster>(new Cluster(variableIndex, {x2, x3}, &graph));

  EXPECT_LONGS_EQUAL(3, marginalCluster->nrFactors());
  EXPECT_LONGS_EQUAL(2, landmarkCluster->nrFactors());
  EXPECT_LONGS_EQUAL(1, rootCluster->nrFactors());

  EXPECT_LONGS_EQUAL(1, marginalCluster->nrFrontals());
  EXPECT_LONGS_EQUAL(2, landmarkCluster->nrFrontals());
  EXPECT_LONGS_EQUAL(2, rootCluster->nrFrontals());

  // Test linearize
  auto gfg = marginalCluster->linearize(initial);
  EXPECT_LONGS_EQUAL(3, gfg->size());

  // Calculate expected result of only evaluating the marginalCluster
  Ordering ordering;
  ordering.push_back(x1);
  const auto [bn, fg] = gfg->eliminatePartialSequential(ordering);
  auto expectedFactor = fg->at<HessianFactor>(0);
  if (!expectedFactor)
    throw std::runtime_error("Expected HessianFactor");

  // Linearize and eliminate the marginalCluster
  auto actual = marginalCluster->linearizeAndEliminate(initial);
  const GaussianBayesNet& bayesNet = actual.first;
  const HessianFactor& factor = *actual.second;
  EXPECT(assert_equal(*bn->at(0), *bayesNet.at(0), 1e-6));
  EXPECT(assert_equal(*expectedFactor, factor, 1e-6));
}

/* ************************************************************************* */
static NonlinearClusterTree Construct() {
  // Build the clusters
  // NOTE(frank): Order matters here as factors are removed!
  NonlinearFactorGraph graph = planarSLAMGraph();
  VariableIndex variableIndex(graph);
  typedef NonlinearClusterTree::NonlinearCluster Cluster;
  auto marginalCluster = std::shared_ptr<Cluster>(new Cluster(variableIndex, {x1}, &graph));
  auto landmarkCluster = std::shared_ptr<Cluster>(new Cluster(variableIndex, {l1, l2}, &graph));
  auto rootCluster = std::shared_ptr<Cluster>(new Cluster(variableIndex, {x2, x3}, &graph));

  // Build the tree
  NonlinearClusterTree clusterTree;
  clusterTree.addRoot(rootCluster);
  rootCluster->addChild(landmarkCluster);
  landmarkCluster->addChild(marginalCluster);

  return clusterTree;
}

/* ************************************************************************* */
TEST(NonlinearClusterTree, Construct) {
  NonlinearClusterTree clusterTree = Construct();

  EXPECT_LONGS_EQUAL(3, clusterTree[0].problemSize());
  EXPECT_LONGS_EQUAL(3, clusterTree[0][0].problemSize());
  EXPECT_LONGS_EQUAL(3, clusterTree[0][0][0].problemSize());
}

/* ************************************************************************* */
TEST(NonlinearClusterTree, Solve) {
  NonlinearClusterTree clusterTree = Construct();

  Values expected;
  expected.insert(l1, Point2(2, 2));
  expected.insert(l2, Point2(4, 2));
  expected.insert(x1, Pose2(0, 0, 0));
  expected.insert(x2, Pose2(2, 0, 0));
  expected.insert(x3, Pose2(4, 0, 0));

  Values values = planarSLAMValues();
  for (size_t i = 0; i < 4; i++)
    values = clusterTree.updateCholesky(values);

  EXPECT(assert_equal(expected, values, 1e-7));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
