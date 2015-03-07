/**
 * @file    testGaussianISAM2.cpp
 * @brief   Unit tests for GaussianISAM2
 * @author  Michael Kaess
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <tests/smallExample.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;
using boost::shared_ptr;

const double tol = 1e-4;

//  SETDEBUG("ISAM2 update", true);
//  SETDEBUG("ISAM2 update verbose", true);
//  SETDEBUG("ISAM2 recalculate", true);

// Set up parameters
SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
SharedDiagonal brNoise = noiseModel::Diagonal::Sigmas(Vector_(2, M_PI/100.0, 0.1));

ISAM2 createSlamlikeISAM2(
		boost::optional<Values&> init_values = boost::none,
		boost::optional<NonlinearFactorGraph&> full_graph = boost::none,
		const ISAM2Params& params = ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false, true)) {

  // These variables will be reused and accumulate factors and values
  ISAM2 isam(params);
  Values fullinit;
  NonlinearFactorGraph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update isam
  {
    NonlinearFactorGraph newfactors;
    newfactors.add(PriorFactor<Pose2>(0, Pose2(0.0, 0.0, 0.0), odoNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert((0), Pose2(0.01, 0.01, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 0 to time 5
  for( ; i<5; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    NonlinearFactorGraph newfactors;
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 100, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 101, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise));
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

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    NonlinearFactorGraph newfactors;
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 101, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert((i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init);
    ++ i;
  }

  if (full_graph)
  	*full_graph = fullgraph;

  if (init_values)
  	*init_values = fullinit;

  return isam;
}

/* ************************************************************************* */
TEST_UNSAFE(ISAM2, ImplAddVariables) {

  // Create initial state
  Values theta;
  theta.insert(0, Pose2(.1, .2, .3));
  theta.insert(100, Point2(.4, .5));
  Values newTheta;
  newTheta.insert(1, Pose2(.6, .7, .8));

  VectorValues delta;
  delta.insert(0, Vector_(3, .1, .2, .3));
  delta.insert(1, Vector_(2, .4, .5));

  VectorValues deltaNewton;
  deltaNewton.insert(0, Vector_(3, .1, .2, .3));
  deltaNewton.insert(1, Vector_(2, .4, .5));

  VectorValues deltaRg;
  deltaRg.insert(0, Vector_(3, .1, .2, .3));
  deltaRg.insert(1, Vector_(2, .4, .5));

  vector<bool> replacedKeys(2, false);

  Ordering ordering; ordering += 100, 0;

  // Verify initial state
  LONGS_EQUAL(0, ordering[100]);
  LONGS_EQUAL(1, ordering[0]);
  EXPECT(assert_equal(delta[0], delta[ordering[100]]));
  EXPECT(assert_equal(delta[1], delta[ordering[0]]));

  // Create expected state
  Values thetaExpected;
  thetaExpected.insert(0, Pose2(.1, .2, .3));
  thetaExpected.insert(100, Point2(.4, .5));
  thetaExpected.insert(1, Pose2(.6, .7, .8));

  VectorValues deltaExpected;
  deltaExpected.insert(0, Vector_(3, .1, .2, .3));
  deltaExpected.insert(1, Vector_(2, .4, .5));
  deltaExpected.insert(2, Vector_(3, 0.0, 0.0, 0.0));

  VectorValues deltaNewtonExpected;
  deltaNewtonExpected.insert(0, Vector_(3, .1, .2, .3));
  deltaNewtonExpected.insert(1, Vector_(2, .4, .5));
  deltaNewtonExpected.insert(2, Vector_(3, 0.0, 0.0, 0.0));

  VectorValues deltaRgExpected;
  deltaRgExpected.insert(0, Vector_(3, .1, .2, .3));
  deltaRgExpected.insert(1, Vector_(2, .4, .5));
  deltaRgExpected.insert(2, Vector_(3, 0.0, 0.0, 0.0));

  vector<bool> replacedKeysExpected(3, false);

  Ordering orderingExpected; orderingExpected += 100, 0, 1;

  // Expand initial state
  ISAM2::Impl::AddVariables(newTheta, theta, delta, deltaNewton, deltaRg, replacedKeys, ordering);

  EXPECT(assert_equal(thetaExpected, theta));
  EXPECT(assert_equal(deltaExpected, delta));
  EXPECT(assert_equal(deltaNewtonExpected, deltaNewton));
  EXPECT(assert_equal(deltaRgExpected, deltaRg));
  EXPECT(assert_container_equality(replacedKeysExpected, replacedKeys));
  EXPECT(assert_equal(orderingExpected, ordering));
}
/* ************************************************************************* */
TEST_UNSAFE(ISAM2, ImplRemoveVariables) {

	// Create initial state
	Values theta;
	theta.insert(0, Pose2(.1, .2, .3));
	theta.insert(1, Pose2(.6, .7, .8));
	theta.insert(100, Point2(.4, .5));

	SymbolicFactorGraph sfg;
	sfg.push_back(boost::make_shared<IndexFactor>(Index(0), Index(2)));
	sfg.push_back(boost::make_shared<IndexFactor>(Index(0), Index(1)));
	VariableIndex variableIndex(sfg);

	VectorValues delta;
	delta.insert(0, Vector_(3, .1, .2, .3));
	delta.insert(1, Vector_(3, .4, .5, .6));
	delta.insert(2, Vector_(2, .7, .8));

	VectorValues deltaNewton;
	deltaNewton.insert(0, Vector_(3, .1, .2, .3));
	deltaNewton.insert(1, Vector_(3, .4, .5, .6));
	deltaNewton.insert(2, Vector_(2, .7, .8));

	VectorValues deltaRg;
	deltaRg.insert(0, Vector_(3, .1, .2, .3));
	deltaRg.insert(1, Vector_(3, .4, .5, .6));
	deltaRg.insert(2, Vector_(2, .7, .8));

	vector<bool> replacedKeys(3, false);
	replacedKeys[0] = true;
	replacedKeys[1] = false;
	replacedKeys[2] = true;

	Ordering ordering; ordering += 100, 1, 0;

	ISAM2::Nodes nodes(3);

	// Verify initial state
	LONGS_EQUAL(0, ordering[100]);
	LONGS_EQUAL(1, ordering[1]);
	LONGS_EQUAL(2, ordering[0]);

	// Create expected state
	Values thetaExpected;
	thetaExpected.insert(0, Pose2(.1, .2, .3));
	thetaExpected.insert(100, Point2(.4, .5));

	SymbolicFactorGraph sfgRemoved;
	sfgRemoved.push_back(boost::make_shared<IndexFactor>(Index(0), Index(1)));
	sfgRemoved.push_back(SymbolicFactorGraph::sharedFactor()); // Add empty factor to keep factor indices consistent
	VariableIndex variableIndexExpected(sfgRemoved);

	VectorValues deltaExpected;
	deltaExpected.insert(0, Vector_(3, .1, .2, .3));
	deltaExpected.insert(1, Vector_(2, .7, .8));

	VectorValues deltaNewtonExpected;
	deltaNewtonExpected.insert(0, Vector_(3, .1, .2, .3));
	deltaNewtonExpected.insert(1, Vector_(2, .7, .8));

	VectorValues deltaRgExpected;
	deltaRgExpected.insert(0, Vector_(3, .1, .2, .3));
	deltaRgExpected.insert(1, Vector_(2, .7, .8));

	vector<bool> replacedKeysExpected(2, true);

	Ordering orderingExpected; orderingExpected += 100, 0;

	ISAM2::Nodes nodesExpected(2);

	// Reduce initial state
	FastSet<Key> unusedKeys;
	unusedKeys.insert(1);
	vector<size_t> removedFactorsI; removedFactorsI.push_back(1);
	SymbolicFactorGraph removedFactors; removedFactors.push_back(sfg[1]);
	variableIndex.remove(removedFactorsI, removedFactors);
	GaussianFactorGraph linearFactors;
	ISAM2::Impl::RemoveVariables(unusedKeys, ISAM2::sharedClique(), theta, variableIndex, delta, deltaNewton, deltaRg,
		replacedKeys, ordering, nodes, linearFactors);

	EXPECT(assert_equal(thetaExpected, theta));
	EXPECT(assert_equal(variableIndexExpected, variableIndex));
	EXPECT(assert_equal(deltaExpected, delta));
	EXPECT(assert_equal(deltaNewtonExpected, deltaNewton));
	EXPECT(assert_equal(deltaRgExpected, deltaRg));
	EXPECT(assert_container_equality(replacedKeysExpected, replacedKeys));
	EXPECT(assert_equal(orderingExpected, ordering));
}

/* ************************************************************************* */
//TEST(ISAM2, IndicesFromFactors) {
//
//  using namespace gtsam::planarSLAM;
//  typedef GaussianISAM2<Values>::Impl Impl;
//
//  Ordering ordering; ordering += (0), (0), (1);
//  NonlinearFactorGraph graph;
//  graph.add(PriorFactor<Pose2>((0), Pose2(), noiseModel::Unit::Create(Pose2::dimension));
//  graph.addRange((0), (0), 1.0, noiseModel::Unit::Create(1));
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
  theta.insert((0), Pose2(0.01, 0.01, 0.01));

  // Create conditional
  Vector d(3); d << -0.1, -0.1, -0.31831;
  Matrix R(3,3); R <<
      10,          0.0,          0.0,
      0.0,           10,          0.0,
      0.0,          0.0,   31.8309886;
  GaussianConditional::shared_ptr conditional(new GaussianConditional(0, d, R, Vector::Ones(3)));

  // Create ordering
  Ordering ordering; ordering += (0);

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
bool isam_check(const NonlinearFactorGraph& fullgraph, const Values& fullinit, const ISAM2& isam, Test& test, TestResult& result) {

	TestResult& result_ = result;
	const std::string name_ = test.getName();

  Values actual = isam.calculateEstimate();
  Ordering ordering = isam.getOrdering(); // *fullgraph.orderingCOLAMD(fullinit).first;
  GaussianFactorGraph linearized = *fullgraph.linearize(fullinit, ordering);
//  linearized.print("Expected linearized: ");
  GaussianBayesNet gbn = *GaussianSequentialSolver(linearized).eliminate();
//  gbn.print("Expected bayesnet: ");
  VectorValues delta = optimize(gbn);
  Values expected = fullinit.retract(delta, ordering);

  bool isamEqual = assert_equal(expected, actual);

	// The following two checks make sure that the cached gradients are maintained and used correctly

	// Check gradient at each node
	bool nodeGradientsOk = true;
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
	VectorValues expectedGradient(*allocateVectorValues(isam));
	gradientAtZero(FactorGraph<JacobianFactor>(isam), expectedGradient);
	VectorValues expectedGradient2(gradient(FactorGraph<JacobianFactor>(isam), VectorValues::Zero(expectedGradient)));
	VectorValues actualGradient(*allocateVectorValues(isam));
	gradientAtZero(isam, actualGradient);
	bool expectedGradOk = assert_equal(expectedGradient2, expectedGradient);
	EXPECT(expectedGradOk);
	bool totalGradOk = assert_equal(expectedGradient, actualGradient);
	EXPECT(totalGradOk);

	return nodeGradientsOk && expectedGradOk && totalGradOk && isamEqual;
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_gaussnewton)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
	ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_dogleg)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
	ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph, ISAM2Params(ISAM2DoglegParams(1.0), 0.0, 0, false));

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_gaussnewton_qr)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
	ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false, false, ISAM2Params::QR));

	// Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution_dogleg_qr)
{
  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
	ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph, ISAM2Params(ISAM2DoglegParams(1.0), 0.0, 0, false, false, ISAM2Params::QR));

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
    factors.add(BetweenFactor<Pose2>(0, 10,
        isam.calculateEstimate<Pose2>(0).between(isam.calculateEstimate<Pose2>(10)), noiseModel::Unit::Create(3)));
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
  // This test builds a graph in the same way as the "slamlike" test above, but
  // then removes the 2nd-to-last landmark measurement

  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
	ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));

	// Remove the 2nd measurement on landmark 0 (Key 100)
	FastVector<size_t> toRemove;
	toRemove.push_back(12);
	isam.update(NonlinearFactorGraph(), Values(), toRemove);

	// Remove the factor from the full system
	fullgraph.remove(12);

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
TEST_UNSAFE(ISAM2, removeVariables)
{
	// These variables will be reused and accumulate factors and values
	Values fullinit;
	NonlinearFactorGraph fullgraph;
	ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph, ISAM2Params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false));

	// Remove the measurement on landmark 0 (Key 100)
	FastVector<size_t> toRemove;
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
TEST_UNSAFE(ISAM2, swapFactors)
{
  // This test builds a graph in the same way as the "slamlike" test above, but
  // then swaps the 2nd-to-last landmark measurement with a different one

  Values fullinit;
  NonlinearFactorGraph fullgraph;
  ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph);

  // Remove the measurement on landmark 0 and replace with a different one
  {
  	size_t swap_idx = isam.getFactorsUnsafe().size()-2;
  	FastVector<size_t> toRemove;
  	toRemove.push_back(swap_idx);
  	fullgraph.remove(swap_idx);

  	NonlinearFactorGraph swapfactors;
//  	swapfactors.add(BearingRange<Pose2,Point2>(10, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise); // original factor
  	swapfactors.add(BearingRangeFactor<Pose2,Point2>(10, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 5.0, brNoise));
  	fullgraph.push_back(swapfactors);
  	isam.update(swapfactors, Values(), toRemove);
  }

  // Compare solutions
  EXPECT(assert_equal(fullgraph, NonlinearFactorGraph(isam.getFactorsUnsafe())));
  EXPECT(isam_check(fullgraph, fullinit, isam, *this, result_));

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
    EXPECT_LONGS_EQUAL(clique->gradientContribution().rows(), variablePosition);
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
    newfactors.add(PriorFactor<Pose2>(0, Pose2(0.0, 0.0, 0.0), odoNoise));
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
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    if(i >= 3)
      isam.update(newfactors, init, FastVector<size_t>(), constrained);
    else
      isam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    NonlinearFactorGraph newfactors;
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 100, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 101, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(1.01, 0.01, 0.01));
    init.insert(100, Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    init.insert(101, Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
    fullinit.insert((i+1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(100, Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
    fullinit.insert(101, Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));

    isam.update(newfactors, init, FastVector<size_t>(), constrained);
    ++ i;
  }

  // Add odometry from time 6 to time 10
  for( ; i<10; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert((i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));

    isam.update(newfactors, init, FastVector<size_t>(), constrained);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    NonlinearFactorGraph newfactors;
    newfactors.add(BetweenFactor<Pose2>(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 100, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
    newfactors.add(BearingRangeFactor<Pose2,Point2>(i, 101, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i+1), Pose2(6.9, 0.1, 0.01));
    fullinit.insert((i+1), Pose2(6.9, 0.1, 0.01));

    isam.update(newfactors, init, FastVector<size_t>(), constrained);
    ++ i;
  }

  // Compare solutions
  EXPECT(isam_check(fullgraph, fullinit, isam, *this, result_));

  // Check that x3 and x4 are last, but either can come before the other
  EXPECT(isam.getOrdering()[(3)] == 12 && isam.getOrdering()[(4)] == 13);

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
TEST(ISAM2, slamlike_solution_partial_relinearization_check)
{

  // These variables will be reused and accumulate factors and values
  Values fullinit;
  NonlinearFactorGraph fullgraph;
	ISAM2Params params(ISAM2GaussNewtonParams(0.001), 0.0, 0, false);
	params.enablePartialRelinearizationCheck = true;
	ISAM2 isam = createSlamlikeISAM2(fullinit, fullgraph, params);

  // Compare solutions
  CHECK(isam_check(fullgraph, fullinit, isam, *this, result_));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
