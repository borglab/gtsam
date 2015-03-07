/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGaussianJunctionTreeB.cpp
 * @date Jul 8, 2010
 * @author nikai
 */

#include <tests/smallExample.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/cholesky.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:
	 C1		 x5 x6 x4
	 C2		  x3 x2 : x4
	 C3		    x1 : x2
	 C4		  x7 : x6
*/
TEST( GaussianJunctionTreeB, constructor2 )
{
	// create a graph
  Ordering ordering; ordering += X(1),X(3),X(5),X(7),X(2),X(6),X(4);
  GaussianFactorGraph fg = createSmoother(7, ordering).first;

	// create an ordering
	GaussianJunctionTree actual(fg);

	vector<Index> frontal1; frontal1 += ordering[X(5)], ordering[X(6)], ordering[X(4)];
	vector<Index> frontal2; frontal2 += ordering[X(3)], ordering[X(2)];
	vector<Index> frontal3; frontal3 += ordering[X(1)];
	vector<Index> frontal4; frontal4 += ordering[X(7)];
	vector<Index> sep1;
	vector<Index> sep2; sep2 += ordering[X(4)];
	vector<Index> sep3; sep3 += ordering[X(2)];
	vector<Index> sep4; sep4 += ordering[X(6)];
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
TEST( GaussianJunctionTreeB, optimizeMultiFrontal )
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
		expected[ordering[X(i)]] = v;
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianJunctionTreeB, optimizeMultiFrontal2)
{
	// create a graph
	example::Graph nlfg = createNonlinearFactorGraph();
	Values noisy = createNoisyValues();
  Ordering ordering; ordering += X(1),X(2),L(1);
	GaussianFactorGraph fg = *nlfg.linearize(noisy, ordering);

	// optimize the graph
	GaussianJunctionTree tree(fg);
	VectorValues actual = tree.optimize(&EliminateQR);

	// verify
	VectorValues expected = createCorrectDelta(ordering); // expected solution
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianJunctionTreeB, slamlike) {
  Values init;
  NonlinearFactorGraph newfactors;
  NonlinearFactorGraph fullgraph;
  SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = noiseModel::Diagonal::Sigmas(Vector_(2, M_PI/100.0, 0.1));

  size_t i = 0;

  newfactors = NonlinearFactorGraph();
  newfactors.add(PriorFactor<Pose2>(X(0), Pose2(0.0, 0.0, 0.0), odoNoise));
  init.insert(X(0), Pose2(0.01, 0.01, 0.01));
  fullgraph.push_back(newfactors);

  for( ; i<5; ++i) {
    newfactors = NonlinearFactorGraph();
    newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
    init.insert(X(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullgraph.push_back(newfactors);
  }

  newfactors = NonlinearFactorGraph();
  newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(0), Rot2::fromAngle(M_PI/4.0), 5.0, brNoise));
  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(1), Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise));
  init.insert(X(i+1), Pose2(1.01, 0.01, 0.01));
  init.insert(L(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
  init.insert(L(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
  fullgraph.push_back(newfactors);
  ++ i;

  for( ; i<5; ++i) {
    newfactors = NonlinearFactorGraph();
    newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
    init.insert(X(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullgraph.push_back(newfactors);
  }

  newfactors = NonlinearFactorGraph();
  newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(0), Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(1), Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
  init.insert(X(i+1), Pose2(6.9, 0.1, 0.01));
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
TEST(GaussianJunctionTreeB, simpleMarginal) {

  typedef BayesTree<GaussianConditional> GaussianBayesTree;

  // Create a simple graph
  NonlinearFactorGraph fg;
  fg.add(PriorFactor<Pose2>(X(0), Pose2(), noiseModel::Isotropic::Sigma(3, 10.0)));
  fg.add(BetweenFactor<Pose2>(X(0), X(1), Pose2(1.0, 0.0, 0.0), noiseModel::Diagonal::Sigmas(Vector_(3, 10.0, 1.0, 1.0))));

  Values init;
  init.insert(X(0), Pose2());
  init.insert(X(1), Pose2(1.0, 0.0, 0.0));

  Ordering ordering;
  ordering += X(1), X(0);

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
  gbt.insert(GaussianJunctionTree(gfg).eliminate(EliminateCholesky));
  marginalFactor = gbt.marginalFactor(1, EliminateCholesky);
  marginalJacobian = boost::dynamic_pointer_cast<JacobianFactor>(marginalFactor);
  Matrix actual3 = inverse(marginalJacobian->getA(marginalJacobian->begin()).transpose() * marginalJacobian->getA(marginalJacobian->begin()));

  EXPECT(assert_equal(expected, actual1));
  EXPECT(assert_equal(expected, actual2));
  EXPECT(assert_equal(expected, actual3));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
