/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGaussianJunctionTree.cpp
 * @date Jul 8, 2010
 * @author nikai
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/base/debug.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/pose2SLAM.h>

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:
	 C1		 x5 x6 x4
	 C2		  x3 x2 : x4
	 C3		    x1 : x2
	 C4		  x7 : x6
*/
TEST( GaussianJunctionTree, constructor2 )
{
	// create a graph
  Ordering ordering; ordering += "x1","x3","x5","x7","x2","x6","x4";
  GaussianFactorGraph fg = createSmoother(7, ordering).first;

	// create an ordering
	GaussianJunctionTree actual(fg);

	vector<Index> frontal1; frontal1 += ordering["x5"], ordering["x6"], ordering["x4"];
	vector<Index> frontal2; frontal2 += ordering["x3"], ordering["x2"];
	vector<Index> frontal3; frontal3 += ordering["x1"];
	vector<Index> frontal4; frontal4 += ordering["x7"];
	vector<Index> sep1;
	vector<Index> sep2; sep2 += ordering["x4"];
	vector<Index> sep3; sep3 += ordering["x2"];
	vector<Index> sep4; sep4 += ordering["x6"];
	EXPECT(assert_equal(frontal1, actual.root()->frontal));
	EXPECT(assert_equal(sep1,     actual.root()->separator));
	LONGS_EQUAL(5,               actual.root()->size());
	list<GaussianJunctionTree::sharedClique>::const_iterator child0it = actual.root()->children().begin();
  list<GaussianJunctionTree::sharedClique>::const_iterator child1it = child0it; ++child1it;
  GaussianJunctionTree::sharedClique child0 = *child0it;
  GaussianJunctionTree::sharedClique child1 = *child1it;
	EXPECT(assert_equal(frontal2, child0->frontal));
	EXPECT(assert_equal(sep2,     child0->separator));
	LONGS_EQUAL(4,               child0->size());
	EXPECT(assert_equal(frontal3, child0->children().front()->frontal));
	EXPECT(assert_equal(sep3,     child0->children().front()->separator));
	LONGS_EQUAL(2,               child0->children().front()->size());
	EXPECT(assert_equal(frontal4, child1->frontal));
	EXPECT(assert_equal(sep4,     child1->separator));
	LONGS_EQUAL(2,               child1->size());
}

/* ************************************************************************* */
TEST( GaussianJunctionTree, optimizeMultiFrontal )
{
	// create a graph
  GaussianFactorGraph fg;
  Ordering ordering;
  boost::tie(fg,ordering) = createSmoother(7);

	// optimize the graph
	GaussianJunctionTree tree(fg);
	VectorValues actual = tree.optimize(&EliminateQR);

	// verify
	VectorValues expected(vector<size_t>(7,2)); // expected solution
	Vector v = Vector_(2, 0., 0.);
	for (int i=1; i<=7; i++)
		expected[ordering[Symbol('x',i)]] = v;
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianJunctionTree, optimizeMultiFrontal2)
{
	// create a graph
	example::Graph nlfg = createNonlinearFactorGraph();
	Values noisy = createNoisyValues();
  Ordering ordering; ordering += "x1","x2","l1";
	GaussianFactorGraph fg = *nlfg.linearize(noisy, ordering);

	// optimize the graph
	GaussianJunctionTree tree(fg);
	VectorValues actual = tree.optimize(&EliminateQR);

	// verify
	VectorValues expected = createCorrectDelta(ordering); // expected solution
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianJunctionTree, slamlike) {
  using planarSLAM::PoseKey;
  using planarSLAM::PointKey;

  Values init;
  planarSLAM::Graph newfactors;
  planarSLAM::Graph fullgraph;
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  size_t i = 0;

  newfactors = planarSLAM::Graph();
  newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
  init.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
  fullgraph.push_back(newfactors);

  for( ; i<5; ++i) {
    newfactors = planarSLAM::Graph();
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullgraph.push_back(newfactors);
  }

  newfactors = planarSLAM::Graph();
  newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
  newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
  newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
  init.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
  init.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
  init.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
  fullgraph.push_back(newfactors);
  ++ i;

  for( ; i<5; ++i) {
    newfactors = planarSLAM::Graph();
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullgraph.push_back(newfactors);
  }

  newfactors = planarSLAM::Graph();
  newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
  newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
  newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
  init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
  fullgraph.push_back(newfactors);
  ++ i;

  // Compare solutions
  Ordering ordering = *fullgraph.orderingCOLAMD(init);
  GaussianFactorGraph linearized = *fullgraph.linearize(init, ordering);

  GaussianJunctionTree gjt(linearized);
  VectorValues deltaactual = gjt.optimize(&EliminateQR);
  Values actual = init.retract(deltaactual, ordering);

  GaussianBayesNet gbn = *GaussianSequentialSolver(linearized).eliminate();
  VectorValues delta = optimize(gbn);
  Values expected = init.retract(delta, ordering);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianJunctionTree, simpleMarginal) {

  typedef BayesTree<GaussianConditional> GaussianBayesTree;

  // Create a simple graph
  pose2SLAM::Graph fg;
  fg.addPrior(0, Pose2(), sharedSigma(3, 10.0));
  fg.addOdometry(0, 1, Pose2(1.0, 0.0, 0.0), sharedSigmas(Vector_(3, 10.0, 1.0, 1.0)));

  Values init;
  init.insert(pose2SLAM::PoseKey(0), Pose2());
  init.insert(pose2SLAM::PoseKey(1), Pose2(1.0, 0.0, 0.0));

  Ordering ordering;
  ordering += "x1", "x0";

  GaussianFactorGraph gfg = *fg.linearize(init, ordering);

  // Compute marginals with both sequential and multifrontal
  Matrix expected = GaussianSequentialSolver(gfg).marginalCovariance(1);

  Matrix actual1 = GaussianMultifrontalSolver(gfg).marginalCovariance(1);
  
  // Compute marginal directly from marginal factor
  GaussianFactor::shared_ptr marginalFactor = GaussianMultifrontalSolver(gfg).marginalFactor(1);
  JacobianFactor::shared_ptr marginalJacobian = boost::dynamic_pointer_cast<JacobianFactor>(marginalFactor);
  Matrix actual2 = inverse(marginalJacobian->getA(marginalJacobian->begin()).transpose() * marginalJacobian->getA(marginalJacobian->begin()));

  // Compute marginal directly from BayesTree
  GaussianBayesTree gbt;
  gbt.insert(GaussianJunctionTree(gfg).eliminate(EliminateLDL));
  marginalFactor = gbt.marginalFactor(1, EliminateLDL);
  marginalJacobian = boost::dynamic_pointer_cast<JacobianFactor>(marginalFactor);
  Matrix actual3 = inverse(marginalJacobian->getA(marginalJacobian->begin()).transpose() * marginalJacobian->getA(marginalJacobian->begin()));

  EXPECT(assert_equal(expected, actual1));
  EXPECT(assert_equal(expected, actual2));
  EXPECT(assert_equal(expected, actual3));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
