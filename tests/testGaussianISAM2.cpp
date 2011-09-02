/**
 * @file    testGaussianISAM2.cpp
 * @brief   Unit tests for GaussianISAM2
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/base/debug.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/nonlinear/GaussianISAM2.h>
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
  planarSLAM::Values theta;
  theta.insert(planarSLAM::PoseKey(0), Pose2(.1, .2, .3));
  theta.insert(planarSLAM::PointKey(0), Point2(.4, .5));
  planarSLAM::Values newTheta;
  newTheta.insert(planarSLAM::PoseKey(1), Pose2(.6, .7, .8));

  VectorValues deltaUnpermuted;
  deltaUnpermuted.reserve(2, 5);
  { Vector a(3); a << .1, .2, .3; deltaUnpermuted.push_back_preallocated(a); }
  { Vector a(2); a << .4, .5; deltaUnpermuted.push_back_preallocated(a); }

  Permutation permutation(2);
  permutation[0] = 1;
  permutation[1] = 0;

  Permuted<VectorValues> delta(permutation, deltaUnpermuted);

  Ordering ordering; ordering += planarSLAM::PointKey(0), planarSLAM::PoseKey(0);

  GaussianISAM2<planarSLAM::Values>::Nodes nodes(2);

  // Verify initial state
  LONGS_EQUAL(0, ordering[planarSLAM::PointKey(0)]);
  LONGS_EQUAL(1, ordering[planarSLAM::PoseKey(0)]);
  EXPECT(assert_equal(deltaUnpermuted[1], delta[ordering[planarSLAM::PointKey(0)]]));
  EXPECT(assert_equal(deltaUnpermuted[0], delta[ordering[planarSLAM::PoseKey(0)]]));

  // Create expected state
  planarSLAM::Values thetaExpected;
  thetaExpected.insert(planarSLAM::PoseKey(0), Pose2(.1, .2, .3));
  thetaExpected.insert(planarSLAM::PointKey(0), Point2(.4, .5));
  thetaExpected.insert(planarSLAM::PoseKey(1), Pose2(.6, .7, .8));

  VectorValues deltaUnpermutedExpected;
  deltaUnpermutedExpected.reserve(3, 8);
  { Vector a(3); a << .1, .2, .3; deltaUnpermutedExpected.push_back_preallocated(a); }
  { Vector a(2); a << .4, .5; deltaUnpermutedExpected.push_back_preallocated(a); }
  { Vector a(3); a << 0, 0, 0; deltaUnpermutedExpected.push_back_preallocated(a); }

  Permutation permutationExpected(3);
  permutationExpected[0] = 1;
  permutationExpected[1] = 0;
  permutationExpected[2] = 2;

  Permuted<VectorValues> deltaExpected(permutationExpected, deltaUnpermutedExpected);

  Ordering orderingExpected; orderingExpected += planarSLAM::PointKey(0), planarSLAM::PoseKey(0), planarSLAM::PoseKey(1);

  GaussianISAM2<planarSLAM::Values>::Nodes nodesExpected(
          3, GaussianISAM2<planarSLAM::Values>::sharedClique());

  // Expand initial state
  GaussianISAM2<planarSLAM::Values>::Impl::AddVariables(newTheta, theta, delta, ordering, nodes);

  EXPECT(assert_equal(thetaExpected, theta));
  EXPECT(assert_equal(deltaUnpermutedExpected, deltaUnpermuted));
  EXPECT(assert_equal(deltaExpected.permutation(), delta.permutation()));
  EXPECT(assert_equal(orderingExpected, ordering));
}

/* ************************************************************************* */
TEST(ISAM2, optimize2) {

  // Create initialization
  planarSLAM::Values theta;
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
  conditional->rhs(expected);
  conditional->solveInPlace(expected);

  // Clique
  GaussianISAM2<planarSLAM::Values>::sharedClique clique(new GaussianISAM2<planarSLAM::Values>::Clique(conditional));
  VectorValues actual(theta.dims(ordering));
  conditional->rhs(actual);
  optimize2(clique, actual);

//  expected.print("expected: ");
//  actual.print("actual: ");
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
bool isam_check(const planarSLAM::Graph& fullgraph, const planarSLAM::Values& fullinit, const GaussianISAM2<planarSLAM::Values>& isam) {
  planarSLAM::Values actual = isam.calculateEstimate();
  Ordering ordering = isam.getOrdering(); // *fullgraph.orderingCOLAMD(fullinit).first;
  GaussianFactorGraph linearized = *fullgraph.linearize(fullinit, ordering);
//  linearized.print("Expected linearized: ");
  GaussianBayesNet gbn = *GaussianSequentialSolver(linearized).eliminate();
//  gbn.print("Expected bayesnet: ");
  VectorValues delta = optimize(gbn);
  planarSLAM::Values expected = fullinit.expmap(delta, ordering);

  return assert_equal(expected, actual);
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution)
{

  // Pose and landmark key types from planarSLAM
  typedef planarSLAM::PoseKey PoseKey;
  typedef planarSLAM::PointKey PointKey;

  // Set up parameters
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  // These variables will be reused and accumulate factors and values
  GaussianISAM2<planarSLAM::Values> isam(ISAM2Params(0.001, 0.0, 0, false));
  planarSLAM::Values fullinit;
  planarSLAM::Graph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    planarSLAM::Graph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    planarSLAM::Values init;
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

    planarSLAM::Values init;
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

    planarSLAM::Values init;
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

    planarSLAM::Values init;
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

    planarSLAM::Values init;
    init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init);
    ++ i;
  }

  // Compare solutions
  EXPECT(isam_check(fullgraph, fullinit, isam));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
