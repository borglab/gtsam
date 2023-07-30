/**
 * @file    testGaussianISAM2.cpp
 * @brief   Unit tests for GaussianISAM2
 * @author  Michael Kaess
 */

#include <gtsam/nonlinear/ISAM2.h>

#include <tests/smallExample.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/treeTraversal-inst.h>

#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;
using std::shared_ptr;

static const SharedNoiseModel model;

//  SETDEBUG("ISAM2 update", true);
//  SETDEBUG("ISAM2 update verbose", true);
//  SETDEBUG("ISAM2 recalculate", true);

// Set up parameters
SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, M_PI/100.0).finished());
SharedDiagonal brNoise = noiseModel::Diagonal::Sigmas((Vector(2) << M_PI/100.0, 0.1).finished());

ISAM2 createSlamlikeISAM2(
    Values* init_values = nullptr,
    NonlinearFactorGraph* full_graph = nullptr,
    const ISAM2Params& params = ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0,
                                            0, false, true,
                                            ISAM2Params::CHOLESKY, true,
                                            DefaultKeyFormatter, true),
    size_t maxPoses = 10) {
  // These variables will be reused and accumulate factors and values
  ISAM2 isam(params);
  Values fullinit;
  NonlinearFactorGraph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    NonlinearFactorGraph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert((0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  if(i > maxPoses)
    goto done;

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);

    if(i > maxPoses)
      goto done;
  }

  if(i > maxPoses)
    goto done;

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 100, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 101, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(100, Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(101, Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert((i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(100, Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(101, Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init);
    ++ i;
  }

  if(i > maxPoses)
    goto done;

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);

    if(i > maxPoses)
      goto done;
  }

  if(i > maxPoses)
    goto done;

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 101, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert((i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init);
    ++ i;
  }

done:

  if (full_graph)
    *full_graph = fullgraph;

  if (init_values)
    *init_values = fullinit;

  return isam;
}

/* ************************************************************************* */
//TEST(ISAM2, CheckRelinearization) {
//
//  typedef GaussianISAM2<Values>::Impl Impl;
//
//  // Create values where indices 1 and 3 are above the threshold of 0.1
//  VectorValues values;
//  values.reserve(4, 10);
//  values.push_back_preallocated(Vector2(0.09, 0.09));
//  values.push_back_preallocated(Vector3(0.11, 0.11, 0.09));
//  values.push_back_preallocated(Vector3(0.09, 0.09, 0.09));
//  values.push_back_preallocated(Vector2(0.11, 0.11));
//
//  // Create a permutation
//  Permutation permutation(4);
//  permutation[0] = 2;
//  permutation[1] = 0;
//  permutation[2] = 1;
//  permutation[3] = 3;
//
//  Permuted<VectorValues> permuted(permutation, values);
//
//  // After permutation, the indices above the threshold are 2 and 2
//  KeySet expected;
//  expected.insert(2);
//  expected.insert(3);
//
//  // Indices checked by CheckRelinearization
//  KeySet actual = Impl::CheckRelinearization(permuted, 0.1);
//
//  EXPECT(assert_equal(expected, actual));
//}

/* ************************************************************************* */
struct ConsistencyVisitor
{
  bool consistent;
  const ISAM2& isam;
  ConsistencyVisitor(const ISAM2& isam) :
    consistent(true), isam(isam) {}
  int operator()(const ISAM2::sharedClique& node, int& parentData)
  {
    if(find(isam.roots().begin(), isam.roots().end(), node) == isam.roots().end())
    {
      if(node->parent_.expired())
        consistent = false;
      if(find(node->parent()->children.begin(), node->parent()->children.end(), node) == node->parent()->children.end())
        consistent = false;
    }
    for(Key j: node->conditional()->frontals())
    {
      if(isam.nodes().at(j).get() != node.get())
        consistent = false;
    }
    return 0;
  }
};

/* ************************************************************************* */
bool isam_check(const NonlinearFactorGraph& fullgraph, const Values& fullinit, const ISAM2& isam, Test& test, TestResult& result) {

  TestResult& result_ = result;
  const string name_ = test.getName();

  Values actual = isam.calculateEstimate();
  Values expected = fullinit.retract(fullgraph.linearize(fullinit)->optimize());

  bool isamEqual = assert_equal(expected, actual);

  // Check information
  GaussianFactorGraph isamGraph(isam);
  isamGraph.push_back(isam.roots().front()->cachedFactor_);
  Matrix expectedHessian = fullgraph.linearize(isam.getLinearizationPoint())->augmentedHessian();
  Matrix actualHessian = isamGraph.augmentedHessian();
  expectedHessian.bottomRightCorner(1,1) = actualHessian.bottomRightCorner(1,1);
  bool isamTreeEqual = assert_equal(expectedHessian, actualHessian);

  // Check consistency
  ConsistencyVisitor visitor(isam);
  int data; // Unused
  treeTraversal::DepthFirstForest(isam, data, visitor);
  bool consistent = visitor.consistent;

  // The following two checks make sure that the cached gradients are maintained and used correctly

  // Check gradient at each node
  bool nodeGradientsOk = true;
  for (const auto& [key, clique] : isam.nodes()) {
    // Compute expected gradient
    GaussianFactorGraph jfg;
    jfg.push_back(clique->conditional());
    VectorValues expectedGradient = jfg.gradientAtZero();
    // Compare with actual gradients
    DenseIndex variablePosition = 0;
    for(GaussianConditional::const_iterator jit = clique->conditional()->begin(); jit != clique->conditional()->end(); ++jit) {
      const DenseIndex dim = clique->conditional()->getDim(jit);
      Vector actual = clique->gradientContribution().segment(variablePosition, dim);
      bool gradOk = assert_equal(expectedGradient[*jit], actual);
      EXPECT(gradOk);
      nodeGradientsOk = nodeGradientsOk && gradOk;
      variablePosition += dim;
    }
    bool dimOk = clique->gradientContribution().rows() == variablePosition;
    EXPECT(dimOk);
    nodeGradientsOk = nodeGradientsOk && dimOk;
  }

  // Check gradient
  VectorValues expectedGradient = GaussianFactorGraph(isam).gradientAtZero();
  VectorValues expectedGradient2 = GaussianFactorGraph(isam).gradient(VectorValues::Zero(expectedGradient));
  VectorValues actualGradient = isam.gradientAtZero();
  bool expectedGradOk = assert_equal(expectedGradient2, expectedGradient);
  EXPECT(expectedGradOk);
  bool totalGradOk = assert_equal(expectedGradient, actualGradient);
  EXPECT(totalGradOk);

  return nodeGradientsOk && expectedGradOk && totalGradOk && isamEqual && isamTreeEqual && consistent;
}

/* ************************************************************************* */
TEST(ISAM2, simple)
{
  for(size_t i = 0; i < 10; ++i) {
    // These variables will be reused and accumulate factors and values
    Values fullinit;
    NonlinearFactorGraph fullgraph;
    ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false), i);

    // Compare solutions
    EXPECT(isam_check(fullgraph, fullinit, isam, *this, result_));
  }
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_gaussnewton)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_dogleg)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, ISAM2Params(ISAM2DoglegParams(1.0), 0.0, 0, false));

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_gaussnewton_qr)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false, false, ISAM2Params::QR));

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_dogleg_qr)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, ISAM2Params(ISAM2DoglegParams(1.0), 0.0, 0, false, false, ISAM2Params::QR));

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, clone) {

  ISAM2 clone1;

  {
    ISAM2 isam = createSlamlikeISAM2();
    clone1 = isam;

    ISAM2 clone2(isam);

    // Modify original isam
    NonlinearFactorGraph factors;
    factors.emplace_shared<BetweenFactor<Pose2>>(0, 10,
        isam.calculateEstimate<Pose2>(0).between(isam.calculateEstimate<Pose2>(10)), noiseModel::Unit::Create(3));
    isam.update(factors);

    CHECK(assert_equal(createSlamlikeISAM2(), clone2));
  }

  // This is to (perhaps unsuccessfully) try to currupt unallocated memory referenced
  // if the references in the iSAM2 copy point to the old instance which deleted at
  // the end of the {...} section above.
  ISAM2 temp = createSlamlikeISAM2();

  CHECK(assert_equal(createSlamlikeISAM2(), clone1));
  CHECK(assert_equal(clone1, temp));

  // Check clone empty
  ISAM2 isam;
  clone1 = isam;
  CHECK(assert_equal(ISAM2(), clone1));
}

/* ************************************************************************* */
TEST(ISAM2, removeFactors)
{
  // This test builds a graph in the same way as the "slamlike" test above, but
  // then removes the 2nd-to-last landmark measurement

  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));

  // Remove the 2nd measurement on landmark 0 (Key 100)
  FactorIndices toRemove;
  toRemove.push_back(12);
  isam.update(NonlinearFactorGraph(), Values(), toRemove);

  // Remove the factor from the full system
  fullgraph.remove(12);

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, removeVariables)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));

  // Remove the measurement on landmark 0 (Key 100)
  FactorIndices toRemove;
  toRemove.push_back(7);
  toRemove.push_back(14);
  isam.update(NonlinearFactorGraph(), Values(), toRemove);

  // Remove the factors and variable from the full system
  fullgraph.remove(7);
  fullgraph.remove(14);
  fullinit.erase(100);

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, swapFactors)
{
  // This test builds a graph in the same way as the "slamlike" test above, but
  // then swaps the 2nd-to-last landmark measurement with a different one

  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph);

  // Remove the measurement on landmark 0 and replace with a different one
  {
    size_t swap_idx = isam.getFactorsUnsafe().size()-2;
    FactorIndices toRemove;
    toRemove.push_back(swap_idx);
    fullgraph.remove(swap_idx);

    NonlinearFactorGraph swapfactors;
//    swapfactors += BearingRange<Pose2,Point2>(10, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise; // original factor
    swapfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(10, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 5.0, brNoise);
    fullgraph.push_back(swapfactors);
    isam.update(swapfactors, Values(), toRemove);
  }

  // Compare solutions
  EXPECT(assert_equal(fullgraph, NonlinearFactorGraph(isam.getFactorsUnsafe())));
  EXPECT(isam_check(fullgraph, fullinit, isam, *this, result_));

  // Check gradient at each node
  for (const auto& [key, clique]: isam.nodes()) {
    // Compute expected gradient
    GaussianFactorGraph jfg;
    jfg.push_back(clique->conditional());
    VectorValues expectedGradient = jfg.gradientAtZero();
    // Compare with actual gradients
    DenseIndex variablePosition = 0;
    for(GaussianConditional::const_iterator jit = clique->conditional()->begin(); jit != clique->conditional()->end(); ++jit) {
      const DenseIndex dim = clique->conditional()->getDim(jit);
      Vector actual = clique->gradientContribution().segment(variablePosition, dim);
      EXPECT(assert_equal(expectedGradient[*jit], actual));
      variablePosition += dim;
    }
    EXPECT_LONGS_EQUAL((long)clique->gradientContribution().rows(), (long)variablePosition);
  }

  // Check gradient
  VectorValues expectedGradient = GaussianFactorGraph(isam).gradientAtZero();
  VectorValues expectedGradient2 = GaussianFactorGraph(isam).gradient(VectorValues::Zero(expectedGradient));
  VectorValues actualGradient = isam.gradientAtZero();
  EXPECT(assert_equal(expectedGradient2, expectedGradient));
  EXPECT(assert_equal(expectedGradient, actualGradient));
}

/* ************************************************************************* */
TEST(ISAM2, constrained_ordering)
{
  // These variables will be reused and accumulate factors and values
  ISAM2 isam(ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));
  Values fullinit;
  NonlinearFactorGraph fullgraph;

  // We will constrain x3 and x4 to the end
  FastMap<Key, int> constrained;
  constrained.insert(make_pair((3), 1));
  constrained.insert(make_pair((4), 2));

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    NonlinearFactorGraph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert((0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    if(i >= 3)
      isam.update(newfactors, init, FactorIndices(), constrained);
    else
      isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 100, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 101, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(100, Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(101, Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert((i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(100, Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(101, Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init, FactorIndices(), constrained);
    ++ i;
  }

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init, FactorIndices(), constrained);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    newfactors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(i, 101, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert((i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init, FactorIndices(), constrained);
    ++ i;
  }

  // Compare solutions
  EXPECT(isam_check(fullgraph, fullinit, isam, *this, result_));

  // Check gradient at each node
  for (const auto& [key, clique]: isam.nodes()) {
    // Compute expected gradient
    GaussianFactorGraph jfg;
    jfg.push_back(clique->conditional());
    VectorValues expectedGradient = jfg.gradientAtZero();
    // Compare with actual gradients
    DenseIndex variablePosition = 0;
    for(GaussianConditional::const_iterator jit = clique->conditional()->begin(); jit != clique->conditional()->end(); ++jit) {
      const DenseIndex dim = clique->conditional()->getDim(jit);
      Vector actual = clique->gradientContribution().segment(variablePosition, dim);
      EXPECT(assert_equal(expectedGradient[*jit], actual));
      variablePosition += dim;
    }
    LONGS_EQUAL((long)clique->gradientContribution().rows(), (long)variablePosition);
  }

  // Check gradient
  VectorValues expectedGradient = GaussianFactorGraph(isam).gradientAtZero();
  VectorValues expectedGradient2 = GaussianFactorGraph(isam).gradient(VectorValues::Zero(expectedGradient));
  VectorValues actualGradient = isam.gradientAtZero();
  EXPECT(assert_equal(expectedGradient2, expectedGradient));
  EXPECT(assert_equal(expectedGradient, actualGradient));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_partial_relinearization_check)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2Params params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false);
  params.enablePartialRelinearizationCheck = true;
  ISAM2 isam = createSlamlikeISAM2(&fullinit, &fullgraph, params);

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

namespace {
  bool checkMarginalizeLeaves(ISAM2& isam, const FastList<Key>& leafKeys) {
    Matrix expectedAugmentedHessian, expected3AugmentedHessian;
    KeyVector toKeep;
    for (const auto& [key, clique]: isam.getDelta()) {
      if(find(leafKeys.begin(), leafKeys.end(), key) == leafKeys.end()) {
        toKeep.push_back(key);
      }
    }

    // Calculate expected marginal from iSAM2 tree
    expectedAugmentedHessian = GaussianFactorGraph(isam).marginal(toKeep, EliminateQR)->augmentedHessian();

    // Calculate expected marginal from cached linear factors
    //assert(isam.params().cacheLinearizedFactors);
    //Matrix expected2AugmentedHessian = isam.linearFactors_.marginal(toKeep, EliminateQR)->augmentedHessian();

    // Calculate expected marginal from original nonlinear factors
    expected3AugmentedHessian = isam.getFactorsUnsafe().linearize(isam.getLinearizationPoint())
      ->marginal(toKeep, EliminateQR)->augmentedHessian();

    // Do marginalization
    isam.marginalizeLeaves(leafKeys);

    // Check
    GaussianFactorGraph actualMarginalGraph(isam);
    Matrix actualAugmentedHessian = actualMarginalGraph.augmentedHessian();
    //Matrix actual2AugmentedHessian = linearFactors_.augmentedHessian();
    Matrix actual3AugmentedHessian = isam.getFactorsUnsafe().linearize(
      isam.getLinearizationPoint())->augmentedHessian();
    assert(actualAugmentedHessian.allFinite());

    // Check full marginalization
    //cout << "treeEqual" << endl;
    bool treeEqual = assert_equal(expectedAugmentedHessian, actualAugmentedHessian, 1e-6);
    //actualAugmentedHessian.bottomRightCorner(1,1) = expected2AugmentedHessian.bottomRightCorner(1,1); bool linEqual = assert_equal(expected2AugmentedHessian, actualAugmentedHessian, 1e-6);
    //cout << "nonlinEqual" << endl;
    actualAugmentedHessian.bottomRightCorner(1,1) = expected3AugmentedHessian.bottomRightCorner(1,1); bool nonlinEqual = assert_equal(expected3AugmentedHessian, actualAugmentedHessian, 1e-6);
    //bool linCorrect = assert_equal(expected3AugmentedHessian, expected2AugmentedHessian, 1e-6);
    //actual2AugmentedHessian.bottomRightCorner(1,1) = expected3AugmentedHessian.bottomRightCorner(1,1); bool afterLinCorrect = assert_equal(expected3AugmentedHessian, actual2AugmentedHessian, 1e-6);
    //cout << "nonlinCorrect" << endl;
    bool afterNonlinCorrect = assert_equal(expected3AugmentedHessian, actual3AugmentedHessian, 1e-6);

    bool ok = treeEqual && /*linEqual &&*/ nonlinEqual && /*linCorrect &&*/ /*afterLinCorrect &&*/ afterNonlinCorrect;
    return ok;
  }

  boost::optional<FastMap<Key, int>> createOrderingConstraints(const ISAM2& isam, const KeyVector& newKeys, const KeySet& marginalizableKeys)
  {
    if (marginalizableKeys.empty()) {
      return boost::none;
    } else {
      FastMap<Key, int> constrainedKeys = FastMap<Key, int>();
      // Generate ordering constraints so that the marginalizable variables will be eliminated first
      // Set all existing and new variables to Group1
      for (const auto& key_val : isam.getDelta()) {
        constrainedKeys.emplace(key_val.first, 1);
      }
      for (const auto& key : newKeys) {
        constrainedKeys.emplace(key, 1);
      }
      // And then re-assign the marginalizable variables to Group0 so that they'll all be leaf nodes
      for (const auto& key : marginalizableKeys) {
        constrainedKeys.at(key) = 0;
      }
      return constrainedKeys;
    }
  }

  void markAffectedKeys(const Key& key, const ISAM2Clique::shared_ptr& rootClique, KeyList& additionalKeys)
  {
    std::stack<ISAM2Clique::shared_ptr> frontier;
    frontier.push(rootClique);
    // Basic DFS to find additional keys
    while (!frontier.empty()) {
      // Get the top of the stack
      const ISAM2Clique::shared_ptr clique = frontier.top();
      frontier.pop();
      // Check if we have more keys and children to add
      if (std::find(clique->conditional()->beginParents(), clique->conditional()->endParents(), key) !=
          clique->conditional()->endParents()) {
        for (Key i : clique->conditional()->frontals()) {
          additionalKeys.push_back(i);
        }
        for (const ISAM2Clique::shared_ptr& child : clique->children) {
          frontier.push(child);
        }
      }
    }
  }

  bool updateAndMarginalize(const NonlinearFactorGraph& newFactors, const Values& newValues, const KeySet& marginalizableKeys, ISAM2& isam)
  {
    // Force ISAM2 to put marginalizable variables at the beginning
    const boost::optional<FastMap<Key, int>> orderingConstraints = createOrderingConstraints(isam, newValues.keys(), marginalizableKeys);

    // Mark additional keys between the marginalized keys and the leaves
    KeyList markedKeys;
    for (Key key : marginalizableKeys) {
      markedKeys.push_back(key);
      ISAM2Clique::shared_ptr clique = isam[key];
      for (const ISAM2Clique::shared_ptr& child : clique->children) {
        markAffectedKeys(key, child, markedKeys);
      }
    }

    // Update
    isam.update(newFactors, newValues, FactorIndices{}, orderingConstraints, boost::none, markedKeys);

    if (!marginalizableKeys.empty()) {
      FastList<Key> leafKeys(marginalizableKeys.begin(), marginalizableKeys.end());
      return checkMarginalizeLeaves(isam, leafKeys);
    }
    else {
      return true;
    }
  }
}

/* ************************************************************************* */
TEST(ISAM2, marginalizeLeaves1) {
  ISAM2 isam;
  NonlinearFactorGraph factors;
  factors.addPrior(0, 0.0, model);

  factors.emplace_shared<BetweenFactor<double>>(0, 1, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(1, 2, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(0, 2, 0.0, model);

  Values values;
  values.insert(0, 0.0);
  values.insert(1, 0.0);
  values.insert(2, 0.0);

  FastMap<Key, int> constrainedKeys;
  constrainedKeys.insert(make_pair(0, 0));
  constrainedKeys.insert(make_pair(1, 1));
  constrainedKeys.insert(make_pair(2, 2));

  isam.update(factors, values, FactorIndices(), constrainedKeys);

  FastList<Key> leafKeys {0};
  EXPECT(checkMarginalizeLeaves(isam, leafKeys));
}

/* ************************************************************************* */
TEST(ISAM2, marginalizeLeaves2) {
  ISAM2 isam;

  NonlinearFactorGraph factors;
  factors.addPrior(0, 0.0, model);

  factors.emplace_shared<BetweenFactor<double>>(0, 1, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(1, 2, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(0, 2, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(2, 3, 0.0, model);

  Values values;
  values.insert(0, 0.0);
  values.insert(1, 0.0);
  values.insert(2, 0.0);
  values.insert(3, 0.0);

  FastMap<Key, int> constrainedKeys;
  constrainedKeys.insert(make_pair(0, 0));
  constrainedKeys.insert(make_pair(1, 1));
  constrainedKeys.insert(make_pair(2, 2));
  constrainedKeys.insert(make_pair(3, 3));

  isam.update(factors, values, FactorIndices(), constrainedKeys);

  FastList<Key> leafKeys {0};
  EXPECT(checkMarginalizeLeaves(isam, leafKeys));
}

/* ************************************************************************* */
TEST(ISAM2, marginalizeLeaves3) {
  ISAM2 isam;

  NonlinearFactorGraph factors;
  factors.addPrior(0, 0.0, model);

  factors.emplace_shared<BetweenFactor<double>>(0, 1, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(1, 2, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(0, 2, 0.0, model);

  factors.emplace_shared<BetweenFactor<double>>(2, 3, 0.0, model);

  factors.emplace_shared<BetweenFactor<double>>(3, 4, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(4, 5, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(3, 5, 0.0, model);

  Values values;
  values.insert(0, 0.0);
  values.insert(1, 0.0);
  values.insert(2, 0.0);
  values.insert(3, 0.0);
  values.insert(4, 0.0);
  values.insert(5, 0.0);

  FastMap<Key, int> constrainedKeys;
  constrainedKeys.insert(make_pair(0, 0));
  constrainedKeys.insert(make_pair(1, 1));
  constrainedKeys.insert(make_pair(2, 2));
  constrainedKeys.insert(make_pair(3, 3));
  constrainedKeys.insert(make_pair(4, 4));
  constrainedKeys.insert(make_pair(5, 5));

  isam.update(factors, values, FactorIndices(), constrainedKeys);

  FastList<Key> leafKeys {0};
  EXPECT(checkMarginalizeLeaves(isam, leafKeys));
}

/* ************************************************************************* */
TEST(ISAM2, marginalizeLeaves4) {
  ISAM2 isam;

  NonlinearFactorGraph factors;
  factors.addPrior(0, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(0, 2, 0.0, model);
  factors.emplace_shared<BetweenFactor<double>>(1, 2, 0.0, model);

  Values values;
  values.insert(0, 0.0);
  values.insert(1, 0.0);
  values.insert(2, 0.0);

  FastMap<Key, int> constrainedKeys;
  constrainedKeys.insert(make_pair(0, 0));
  constrainedKeys.insert(make_pair(1, 1));
  constrainedKeys.insert(make_pair(2, 2));

  isam.update(factors, values, FactorIndices(), constrainedKeys);

  FastList<Key> leafKeys {1};
  EXPECT(checkMarginalizeLeaves(isam, leafKeys));
}

/* ************************************************************************* */
TEST(ISAM2, marginalizeLeaves5)
{
  // Create isam2
  ISAM2 isam = createSlamlikeISAM2();

  // Marginalize
  FastList<Key> marginalizeKeys {0};
  EXPECT(checkMarginalizeLeaves(isam, marginalizeKeys));
}

/* ************************************************************************* */
TEST(ISAM2, marginalizeLeaves6)
{
  const boost::shared_ptr<noiseModel::Isotropic> nm = noiseModel::Isotropic::Sigma(6, 1.0);

  int gridDim = 10;

  auto idxToKey = [gridDim](int i, int j){return i * gridDim + j;};

  NonlinearFactorGraph factors;
  Values values;
  ISAM2 isam;

  // Create a grid of pose variables
  for (int i = 0; i < gridDim; ++i) {
    for (int j = 0; j < gridDim; ++j) {
      Pose3 pose = Pose3(Rot3::identity(), Point3(i, j, 0));
      Key key = idxToKey(i, j);
      // Place a prior on the first pose
      factors.addPrior(key, Pose3(Rot3::identity(), Point3(i, j, 0)), nm);
      values.insert(key, pose);
      if (i > 0) {
        factors.emplace_shared<BetweenFactor<Pose3>>(idxToKey(i - 1, j), key, Pose3(Rot3::identity(), Point3(1, 0, 0)),nm);
      }
      if (j > 0) {
        factors.emplace_shared<BetweenFactor<Pose3>>(idxToKey(i, j - 1), key, Pose3(Rot3::identity(), Point3(0, 1, 0)),nm);
      }
    }
  }

  // Optimize the graph
  EXPECT(updateAndMarginalize(factors, values, {}, isam));
  auto estimate = isam.calculateBestEstimate();

  // Get the list of keys
  std::vector<Key> key_list(gridDim * gridDim);
  std::iota(key_list.begin(), key_list.end(), 0);

  // Shuffle the keys -> we will marginalize the keys one by one in this order
  std::shuffle(key_list.begin(), key_list.end(), std::default_random_engine(1234));

  for (const auto& key : key_list) {
    KeySet marginalKeys;
    marginalKeys.insert(key);
    EXPECT(updateAndMarginalize({}, {}, marginalKeys, isam));
    estimate = isam.calculateBestEstimate();
  }
}

/* ************************************************************************* */
TEST(ISAM2, MarginalizeRoot)
{
  const boost::shared_ptr<noiseModel::Isotropic> nm = noiseModel::Isotropic::Sigma(6, 1.0);

  NonlinearFactorGraph factors;
  Values values;
  ISAM2 isam;

  // Create a factor graph with one variable and a prior
  Pose3 root = Pose3::identity();
  Key rootKey(0);
  values.insert(rootKey, root);
  factors.addPrior(rootKey, Pose3::identity(), nm);

  // Optimize the graph
  EXPECT(updateAndMarginalize(factors, values, {}, isam));
  auto estimate = isam.calculateBestEstimate();
  EXPECT(estimate.size() == 1);

  // And remove the node from the graph
  KeySet marginalizableKeys;
  marginalizableKeys.insert(rootKey);

  EXPECT(updateAndMarginalize({}, {}, marginalizableKeys, isam));

  estimate = isam.calculateBestEstimate();
  EXPECT(estimate.empty());
}

/* ************************************************************************* */
TEST(ISAM2, marginalizationSize)
{
  const boost::shared_ptr<noiseModel::Isotropic> nm = noiseModel::Isotropic::Sigma(6, 1.0);

  NonlinearFactorGraph factors;
  Values values;
  ISAM2Params params;
  params.findUnusedFactorSlots = true;
  ISAM2 isam{params};

  // Create a pose variable
  Key aKey(0);
  values.insert(aKey, Pose3::identity());
  factors.addPrior(aKey, Pose3::identity(), nm);
  // Create another pose variable linked to the first
  Pose3 b = Pose3::identity();
  Key bKey(1);
  values.insert(bKey, Pose3::identity());
  factors.emplace_shared<BetweenFactor<Pose3>>(aKey, bKey, Pose3::identity(), nm);
  // Optimize the graph
  EXPECT(updateAndMarginalize(factors, values, {}, isam));

  // Now remove a variable -> we should not see the number of factors increase
  gtsam::KeySet to_remove;
  to_remove.insert(aKey);
  const auto numFactorsBefore = isam.getFactorsUnsafe().size();
  updateAndMarginalize({}, {}, to_remove, isam);
  EXPECT(numFactorsBefore == isam.getFactorsUnsafe().size());
}

/* ************************************************************************* */
TEST(ISAM2, marginalCovariance)
{
  // Create isam2
  ISAM2 isam = createSlamlikeISAM2();

  // Check marginal
  Matrix expected = Marginals(isam.getFactorsUnsafe(), isam.getLinearizationPoint()).marginalCovariance(5);
  Matrix actual = isam.marginalCovariance(5);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(ISAM2, calculate_nnz)
{
  ISAM2 isam = createSlamlikeISAM2();
  int expected = 241;
  int actual = isam.roots().front()->calculate_nnz();

  EXPECT_LONGS_EQUAL(expected, actual);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
