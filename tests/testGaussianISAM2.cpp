/**
 * @file    testGaussianISAM2.cpp
 * @brief   Unit tests for GaussianISAM2
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/debug.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/planarSLAM.h>

using namespace std;
using namespace gtsam;
using namespace example;
using boost::shared_ptr;

const double tol = 1e-4;

/* ************************************************************************* */
TEST(ISAM2, AddVariables) {

  // Create initial state
  Values theta;
  theta.insert(planarSLAM::PoseKey(0), Pose2(.1, .2, .3));
  theta.insert(planarSLAM::PointKey(0), Point2(.4, .5));
  Values newTheta;
  newTheta.insert(planarSLAM::PoseKey(1), Pose2(.6, .7, .8));

  VectorValues deltaUnpermuted;
  deltaUnpermuted.insert(0, Vector_(3, .1, .2, .3));
  deltaUnpermuted.insert(1, Vector_(2, .4, .5));

  Permutation permutation(2);
  permutation[0] = 1;
  permutation[1] = 0;

  Permuted<VectorValues> delta(permutation, deltaUnpermuted);

  vector<bool> replacedKeys(2, false);

  Ordering ordering; ordering += planarSLAM::PointKey(0), planarSLAM::PoseKey(0);

  ISAM2::Nodes nodes(2);

  // Verify initial state
  LONGS_EQUAL(0, ordering[planarSLAM::PointKey(0)]);
  LONGS_EQUAL(1, ordering[planarSLAM::PoseKey(0)]);
  EXPECT(assert_equal(deltaUnpermuted[1], delta[ordering[planarSLAM::PointKey(0)]]));
  EXPECT(assert_equal(deltaUnpermuted[0], delta[ordering[planarSLAM::PoseKey(0)]]));

  // Create expected state
  Values thetaExpected;
  thetaExpected.insert(planarSLAM::PoseKey(0), Pose2(.1, .2, .3));
  thetaExpected.insert(planarSLAM::PointKey(0), Point2(.4, .5));
  thetaExpected.insert(planarSLAM::PoseKey(1), Pose2(.6, .7, .8));

  VectorValues deltaUnpermutedExpected;
  deltaUnpermutedExpected.insert(0, Vector_(3, .1, .2, .3));
  deltaUnpermutedExpected.insert(1, Vector_(2, .4, .5));
  deltaUnpermutedExpected.insert(2, Vector_(3, 0.0, 0.0, 0.0));

  Permutation permutationExpected(3);
  permutationExpected[0] = 1;
  permutationExpected[1] = 0;
  permutationExpected[2] = 2;

  Permuted<VectorValues> deltaExpected(permutationExpected, deltaUnpermutedExpected);

  vector<bool> replacedKeysExpected(3, false);

  Ordering orderingExpected; orderingExpected += planarSLAM::PointKey(0), planarSLAM::PoseKey(0), planarSLAM::PoseKey(1);

  ISAM2::Nodes nodesExpected(
          3, ISAM2::sharedClique());

  // Expand initial state
  ISAM2::Impl::AddVariables(newTheta, theta, delta, replacedKeys, ordering, nodes);

  EXPECT(assert_equal(thetaExpected, theta));
  EXPECT(assert_equal(deltaUnpermutedExpected, deltaUnpermuted));
  EXPECT(assert_equal(deltaExpected.permutation(), delta.permutation()));
  EXPECT(assert_container_equality(replacedKeysExpected, replacedKeys));
  EXPECT(assert_equal(orderingExpected, ordering));
}

/* ************************************************************************* */
//TEST(ISAM2, IndicesFromFactors) {
//
//  using namespace gtsam::planarSLAM;
//  typedef GaussianISAM2<Values>::Impl Impl;
//
//  Ordering ordering; ordering += PointKey(0), PoseKey(0), PoseKey(1);
//  planarSLAM::Graph graph;
//  graph.addPrior(PoseKey(0), Pose2(), sharedUnit(Pose2::dimension));
//  graph.addRange(PoseKey(0), PointKey(0), 1.0, sharedUnit(1));
//
//  FastSet<Index> expected;
//  expected.insert(0);
//  expected.insert(1);
//
//  FastSet<Index> actual = Impl::IndicesFromFactors(ordering, graph);
//
//  EXPECT(assert_equal(expected, actual));
//}

/* ************************************************************************* */
//TEST(ISAM2, CheckRelinearization) {
//
//  typedef GaussianISAM2<Values>::Impl Impl;
//
//  // Create values where indices 1 and 3 are above the threshold of 0.1
//  VectorValues values;
//  values.reserve(4, 10);
//  values.push_back_preallocated(Vector_(2, 0.09, 0.09));
//  values.push_back_preallocated(Vector_(3, 0.11, 0.11, 0.09));
//  values.push_back_preallocated(Vector_(3, 0.09, 0.09, 0.09));
//  values.push_back_preallocated(Vector_(2, 0.11, 0.11));
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
//  FastSet<Index> expected;
//  expected.insert(2);
//  expected.insert(3);
//
//  // Indices checked by CheckRelinearization
//  FastSet<Index> actual = Impl::CheckRelinearization(permuted, 0.1);
//
//  EXPECT(assert_equal(expected, actual));
//}

/* ************************************************************************* */
TEST(ISAM2, optimize2) {

  // Create initialization
  Values theta;
  theta.insert(planarSLAM::PoseKey(0), Pose2(0.01, 0.01, 0.01));

  // Create conditional
  Vector d(3); d << -0.1, -0.1, -0.31831;
  Matrix R(3,3); R <<
      10,          0.0,          0.0,
      0.0,           10,          0.0,
      0.0,          0.0,   31.8309886;
  GaussianConditional::shared_ptr conditional(new GaussianConditional(0, d, R, Vector::Ones(3)));

  // Create ordering
  Ordering ordering; ordering += planarSLAM::PoseKey(0);

  // Expected vector
  VectorValues expected(1, 3);
  conditional->solveInPlace(expected);

  // Clique
  ISAM2::sharedClique clique(
      ISAM2::Clique::Create(make_pair(conditional,GaussianFactor::shared_ptr())));
  VectorValues actual(theta.dims(ordering));
  internal::optimizeInPlace<ISAM2::Base>(clique, actual);

//  expected.print("expected: ");
//  actual.print("actual: ");
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
bool isam_check(const planarSLAM::Graph& fullgraph, const Values& fullinit, const ISAM2& isam) {
  Values actual = isam.calculateEstimate();
  Ordering ordering = isam.getOrdering(); // *fullgraph.orderingCOLAMD(fullinit).first;
  GaussianFactorGraph linearized = *fullgraph.linearize(fullinit, ordering);
//  linearized.print("Expected linearized: ");
  GaussianBayesNet gbn = *GaussianSequentialSolver(linearized).eliminate();
//  gbn.print("Expected bayesnet: ");
  VectorValues delta = optimize(gbn);
  Values expected = fullinit.retract(delta, ordering);

  return assert_equal(expected, actual);
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_gaussnewton)
{

//  SETDEBUG("ISAM2 update", true);
//  SETDEBUG("ISAM2 update verbose", true);
//  SETDEBUG("ISAM2 recalculate", true);

  // Pose and landmark key types from planarSLAM
  using planarSLAM::PoseKey;
  using planarSLAM::PointKey;

  // Set up parameters
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  // These variables will be reused and accumulate factors and values
  ISAM2 isam(ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));
  Values fullinit;
  planarSLAM::Graph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    planarSLAM::Graph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  CHECK(isam_check(fullgraph, fullinit, isam));

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init);
    ++ i;
  }

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init);
    ++ i;
  }

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam));

  // Check gradient at each node
  typedef ISAM2::sharedClique sharedClique;
  BOOST_FOREACH(const sharedClique& clique, isam.nodes()) {
    // Compute expected gradient
    FactorGraph<JacobianFactor> jfg;
    jfg.push_back(JacobianFactor::shared_ptr(new JacobianFactor(*clique->conditional())));
    VectorValues expectedGradient(*allocateVectorValues(isam));
    gradientAtZero(jfg, expectedGradient);
    // Compare with actual gradients
    int variablePosition = 0;
    for(GaussianConditional::const_iterator jit = clique->conditional()->begin(); jit != clique->conditional()->end(); ++jit) {
      const int dim = clique->conditional()->dim(jit);
      Vector actual = clique->gradientContribution().segment(variablePosition, dim);
      EXPECT(assert_equal(expectedGradient[*jit], actual));
      variablePosition += dim;
    }
    LONGS_EQUAL(clique->gradientContribution().rows(), variablePosition);
  }

  // Check gradient
  VectorValues expectedGradient(*allocateVectorValues(isam));
  gradientAtZero(FactorGraph<JacobianFactor>(isam), expectedGradient);
  VectorValues expectedGradient2(gradient(FactorGraph<JacobianFactor>(isam), VectorValues::Zero(expectedGradient)));
  VectorValues actualGradient(*allocateVectorValues(isam));
  gradientAtZero(isam, actualGradient);
  EXPECT(assert_equal(expectedGradient2, expectedGradient));
  EXPECT(assert_equal(expectedGradient, actualGradient));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_dogleg)
{

//  SETDEBUG("ISAM2 update", true);
//  SETDEBUG("ISAM2 update verbose", true);
//  SETDEBUG("ISAM2 recalculate", true);

  // Pose and landmark key types from planarSLAM
  using planarSLAM::PoseKey;
  using planarSLAM::PointKey;

  // Set up parameters
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  // These variables will be reused and accumulate factors and values
  ISAM2 isam(ISAM2Params(ISAM2DoglegParams(1.0), 0.0, 0, false));
  Values fullinit;
  planarSLAM::Graph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    planarSLAM::Graph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  CHECK(isam_check(fullgraph, fullinit, isam));

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init);
    ++ i;
  }

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init);
    ++ i;
  }

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam));

  // Check gradient at each node
  typedef ISAM2::sharedClique sharedClique;
  BOOST_FOREACH(const sharedClique& clique, isam.nodes()) {
    // Compute expected gradient
    FactorGraph<JacobianFactor> jfg;
    jfg.push_back(JacobianFactor::shared_ptr(new JacobianFactor(*clique->conditional())));
    VectorValues expectedGradient(*allocateVectorValues(isam));
    gradientAtZero(jfg, expectedGradient);
    // Compare with actual gradients
    int variablePosition = 0;
    for(GaussianConditional::const_iterator jit = clique->conditional()->begin(); jit != clique->conditional()->end(); ++jit) {
      const int dim = clique->conditional()->dim(jit);
      Vector actual = clique->gradientContribution().segment(variablePosition, dim);
      EXPECT(assert_equal(expectedGradient[*jit], actual));
      variablePosition += dim;
    }
    LONGS_EQUAL(clique->gradientContribution().rows(), variablePosition);
  }

  // Check gradient
  VectorValues expectedGradient(*allocateVectorValues(isam));
  gradientAtZero(FactorGraph<JacobianFactor>(isam), expectedGradient);
  VectorValues expectedGradient2(gradient(FactorGraph<JacobianFactor>(isam), VectorValues::Zero(expectedGradient)));
  VectorValues actualGradient(*allocateVectorValues(isam));
  gradientAtZero(isam, actualGradient);
  EXPECT(assert_equal(expectedGradient2, expectedGradient));
  EXPECT(assert_equal(expectedGradient, actualGradient));
}

/* ************************************************************************* */
TEST(ISAM2, clone) {

  // Pose and landmark key types from planarSLAM
  using planarSLAM::PoseKey;
  using planarSLAM::PointKey;

  // Set up parameters
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  // These variables will be reused and accumulate factors and values
  ISAM2 isam(ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false, true));
  Values fullinit;
  planarSLAM::Graph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    planarSLAM::Graph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  EXPECT(isam_check(fullgraph, fullinit, isam));

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init);
    ++ i;
  }

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init);
    ++ i;
  }

  // CLONING...
  boost::shared_ptr<ISAM2 > isam2
      = boost::shared_ptr<ISAM2 >(new ISAM2());
  isam.cloneTo(isam2);

  CHECK(assert_equal(isam, *isam2));
}

/* ************************************************************************* */
TEST(ISAM2, permute_cached) {
  typedef boost::shared_ptr<ISAM2Clique> sharedISAM2Clique;

  // Construct expected permuted BayesTree (variable 2 has been changed to 1)
  BayesTree<GaussianConditional, ISAM2Clique> expected;
  expected.insert(sharedISAM2Clique(new ISAM2Clique(make_pair(
      boost::make_shared<GaussianConditional>(pair_list_of
          (3, Matrix_(1,1,1.0))
          (4, Matrix_(1,1,2.0)),
          2, Vector_(1,1.0), Vector_(1,1.0)),   // p(3,4)
      HessianFactor::shared_ptr()))));          // Cached: empty
  expected.insert(sharedISAM2Clique(new ISAM2Clique(make_pair(
      boost::make_shared<GaussianConditional>(pair_list_of
          (2, Matrix_(1,1,1.0))
          (3, Matrix_(1,1,2.0)),
          1, Vector_(1,1.0), Vector_(1,1.0)),     // p(2|3)
      boost::make_shared<HessianFactor>(3, Matrix_(1,1,1.0), Vector_(1,1.0), 0.0))))); // Cached: p(3)
  expected.insert(sharedISAM2Clique(new ISAM2Clique(make_pair(
      boost::make_shared<GaussianConditional>(pair_list_of
          (0, Matrix_(1,1,1.0))
          (2, Matrix_(1,1,2.0)),
          1, Vector_(1,1.0), Vector_(1,1.0)),     // p(0|2)
      boost::make_shared<HessianFactor>(1, Matrix_(1,1,1.0), Vector_(1,1.0), 0.0))))); // Cached: p(1)
  // Change variable 2 to 1
  expected.root()->children().front()->conditional()->keys()[0] = 1;
  expected.root()->children().front()->children().front()->conditional()->keys()[1] = 1;

  // Construct unpermuted BayesTree
  BayesTree<GaussianConditional, ISAM2Clique> actual;
  actual.insert(sharedISAM2Clique(new ISAM2Clique(make_pair(
      boost::make_shared<GaussianConditional>(pair_list_of
          (3, Matrix_(1,1,1.0))
          (4, Matrix_(1,1,2.0)),
          2, Vector_(1,1.0), Vector_(1,1.0)),   // p(3,4)
      HessianFactor::shared_ptr()))));          // Cached: empty
  actual.insert(sharedISAM2Clique(new ISAM2Clique(make_pair(
      boost::make_shared<GaussianConditional>(pair_list_of
          (2, Matrix_(1,1,1.0))
          (3, Matrix_(1,1,2.0)),
          1, Vector_(1,1.0), Vector_(1,1.0)),     // p(2|3)
      boost::make_shared<HessianFactor>(3, Matrix_(1,1,1.0), Vector_(1,1.0), 0.0))))); // Cached: p(3)
  actual.insert(sharedISAM2Clique(new ISAM2Clique(make_pair(
      boost::make_shared<GaussianConditional>(pair_list_of
          (0, Matrix_(1,1,1.0))
          (2, Matrix_(1,1,2.0)),
          1, Vector_(1,1.0), Vector_(1,1.0)),     // p(0|2)
      boost::make_shared<HessianFactor>(2, Matrix_(1,1,1.0), Vector_(1,1.0), 0.0))))); // Cached: p(2)

  // Create permutation that changes variable 2 -> 0
  Permutation permutation = Permutation::Identity(5);
  permutation[2] = 1;

  // Permute BayesTree
  actual.root()->permuteWithInverse(permutation);

  // Check
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(ISAM2, removeFactors)
{

//  SETDEBUG("ISAM2 update", true);
//  SETDEBUG("ISAM2 update verbose", true);
//  SETDEBUG("ISAM2 recalculate", true);

  // This test builds a graph in the same way as the "slamlike" test above, but
  // then removes the 2nd-to-last landmark measurement

  // Pose and landmark key types from planarSLAM
  using planarSLAM::PoseKey;
  using planarSLAM::PointKey;

  // Set up parameters
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  // These variables will be reused and accumulate factors and values
  ISAM2 isam(ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));
  Values fullinit;
  planarSLAM::Graph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    planarSLAM::Graph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  CHECK(isam_check(fullgraph, fullinit, isam));

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init);
    ++ i;
  }

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    fullgraph.push_back(newfactors[0]);
    fullgraph.push_back(newfactors[2]); // Don't add measurement on landmark 0

    Values init;
    init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));

    ISAM2Result result = isam.update(newfactors, init);
    ++ i;

    // Remove the measurement on landmark 0
    FastVector<size_t> toRemove;
    toRemove.push_back(result.newFactorsIndices[1]);
    isam.update(planarSLAM::Graph(), Values(), toRemove);
  }

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam));

  // Check gradient at each node
  typedef ISAM2::sharedClique sharedClique;
  BOOST_FOREACH(const sharedClique& clique, isam.nodes()) {
    // Compute expected gradient
    FactorGraph<JacobianFactor> jfg;
    jfg.push_back(JacobianFactor::shared_ptr(new JacobianFactor(*clique->conditional())));
    VectorValues expectedGradient(*allocateVectorValues(isam));
    gradientAtZero(jfg, expectedGradient);
    // Compare with actual gradients
    int variablePosition = 0;
    for(GaussianConditional::const_iterator jit = clique->conditional()->begin(); jit != clique->conditional()->end(); ++jit) {
      const int dim = clique->conditional()->dim(jit);
      Vector actual = clique->gradientContribution().segment(variablePosition, dim);
      EXPECT(assert_equal(expectedGradient[*jit], actual));
      variablePosition += dim;
    }
    LONGS_EQUAL(clique->gradientContribution().rows(), variablePosition);
  }

  // Check gradient
  VectorValues expectedGradient(*allocateVectorValues(isam));
  gradientAtZero(FactorGraph<JacobianFactor>(isam), expectedGradient);
  VectorValues expectedGradient2(gradient(FactorGraph<JacobianFactor>(isam), VectorValues::Zero(expectedGradient)));
  VectorValues actualGradient(*allocateVectorValues(isam));
  gradientAtZero(isam, actualGradient);
  EXPECT(assert_equal(expectedGradient2, expectedGradient));
  EXPECT(assert_equal(expectedGradient, actualGradient));
}

/* ************************************************************************* */
TEST(ISAM2, constrained_ordering)
{

//  SETDEBUG("ISAM2 update", true);
//  SETDEBUG("ISAM2 update verbose", true);
//  SETDEBUG("ISAM2 recalculate", true);

  // Pose and landmark key types from planarSLAM
  using planarSLAM::PoseKey;
  using planarSLAM::PointKey;

  // Set up parameters
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  // These variables will be reused and accumulate factors and values
  ISAM2 isam(ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));
  Values fullinit;
  planarSLAM::Graph fullgraph;

  // We will constrain x3 and x4 to the end
  FastSet<Key> constrained; constrained.insert(planarSLAM::PoseKey(3)); constrained.insert(planarSLAM::PoseKey(4));

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    planarSLAM::Graph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  CHECK(isam_check(fullgraph, fullinit, isam));

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    if(i >= 3)
      isam.update(newfactors, init, FastVector<size_t>(), constrained);
    else
      isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init, FastVector<size_t>(), constrained);
    ++ i;
  }

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init, FastVector<size_t>(), constrained);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    planarSLAM::Graph newfactors;
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init, FastVector<size_t>(), constrained);
    ++ i;
  }

  // Compare solutions
  EXPECT(isam_check(fullgraph, fullinit, isam));

  // Check that x3 and x4 are last, but either can come before the other
  EXPECT((isam.getOrdering()[planarSLAM::PoseKey(3)] == 12 && isam.getOrdering()[planarSLAM::PoseKey(4)] == 13) ||
      (isam.getOrdering()[planarSLAM::PoseKey(3)] == 13 && isam.getOrdering()[planarSLAM::PoseKey(4)] == 12));

  // Check gradient at each node
  typedef ISAM2::sharedClique sharedClique;
  BOOST_FOREACH(const sharedClique& clique, isam.nodes()) {
    // Compute expected gradient
    FactorGraph<JacobianFactor> jfg;
    jfg.push_back(JacobianFactor::shared_ptr(new JacobianFactor(*clique->conditional())));
    VectorValues expectedGradient(*allocateVectorValues(isam));
    gradientAtZero(jfg, expectedGradient);
    // Compare with actual gradients
    int variablePosition = 0;
    for(GaussianConditional::const_iterator jit = clique->conditional()->begin(); jit != clique->conditional()->end(); ++jit) {
      const int dim = clique->conditional()->dim(jit);
      Vector actual = clique->gradientContribution().segment(variablePosition, dim);
      EXPECT(assert_equal(expectedGradient[*jit], actual));
      variablePosition += dim;
    }
    LONGS_EQUAL(clique->gradientContribution().rows(), variablePosition);
  }

  // Check gradient
  VectorValues expectedGradient(*allocateVectorValues(isam));
  gradientAtZero(FactorGraph<JacobianFactor>(isam), expectedGradient);
  VectorValues expectedGradient2(gradient(FactorGraph<JacobianFactor>(isam), VectorValues::Zero(expectedGradient)));
  VectorValues actualGradient(*allocateVectorValues(isam));
  gradientAtZero(isam, actualGradient);
  EXPECT(assert_equal(expectedGradient2, expectedGradient));
  EXPECT(assert_equal(expectedGradient, actualGradient));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
