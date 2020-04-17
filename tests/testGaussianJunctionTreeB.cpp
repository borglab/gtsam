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
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/ClusterTree.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/vector.hpp>

#include <cmath>
#include <list>
#include <utility>
#include <vector>

using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:
 C1     x5 x6 x4
 C2      x3 x2 : x4
 C3        x1 : x2
 C4      x7 : x6
 */
TEST( GaussianJunctionTreeB, constructor2 ) {
  // create a graph
  NonlinearFactorGraph nlfg;
  Values values;
  boost::tie(nlfg, values) = createNonlinearSmoother(7);
  SymbolicFactorGraph::shared_ptr symbolic = nlfg.symbolic();

  // linearize
  GaussianFactorGraph::shared_ptr fg = nlfg.linearize(values);

  Ordering ordering;
  ordering += X(1), X(3), X(5), X(7), X(2), X(6), X(4);

  // create an ordering
  GaussianEliminationTree etree(*fg, ordering);
  SymbolicEliminationTree stree(*symbolic, ordering);
  GaussianJunctionTree actual(etree);

  Ordering o324;
  o324 += X(3), X(2), X(4);
  Ordering o56;
  o56 += X(5), X(6);
  Ordering o7;
  o7 += X(7);
  Ordering o1;
  o1 += X(1);

  // Can no longer test these:
//  Ordering sep1;
//  Ordering sep2; sep2 += X(4);
//  Ordering sep3; sep3 += X(6);
//  Ordering sep4; sep4 += X(2);

  GaussianJunctionTree::sharedNode x324 = actual.roots().front();
  LONGS_EQUAL(2, x324->children.size());
  GaussianJunctionTree::sharedNode x1 = x324->children.front();
  GaussianJunctionTree::sharedNode x56 = x324->children.back();
  if (x1->children.size() > 0)
    x1.swap(x56); // makes it work with different tie-breakers

  LONGS_EQUAL(0, x1->children.size());
  LONGS_EQUAL(1, x56->children.size());
  GaussianJunctionTree::sharedNode x7 = x56->children[0];
  LONGS_EQUAL(0, x7->children.size());

  EXPECT(assert_equal(o324, x324->orderedFrontalKeys));
  EXPECT_LONGS_EQUAL(5, x324->factors.size());
  EXPECT_LONGS_EQUAL(9, x324->problemSize_);

  EXPECT(assert_equal(o56, x56->orderedFrontalKeys));
  EXPECT_LONGS_EQUAL(4, x56->factors.size());
  EXPECT_LONGS_EQUAL(9, x56->problemSize_);

  EXPECT(assert_equal(o7, x7->orderedFrontalKeys));
  EXPECT_LONGS_EQUAL(2, x7->factors.size());
  EXPECT_LONGS_EQUAL(4, x7->problemSize_);

  EXPECT(assert_equal(o1, x1->orderedFrontalKeys));
  EXPECT_LONGS_EQUAL(2, x1->factors.size());
  EXPECT_LONGS_EQUAL(4, x1->problemSize_);
}

///* ************************************************************************* */
//TEST( GaussianJunctionTreeB, optimizeMultiFrontal )
//{
//  // create a graph
//  GaussianFactorGraph fg;
//  Ordering ordering;
//  boost::tie(fg,ordering) = createSmoother(7);
//
//  // optimize the graph
//  GaussianJunctionTree tree(fg);
//  VectorValues actual = tree.optimize(&EliminateQR);
//
//  // verify
//  VectorValues expected(vector<size_t>(7,2)); // expected solution
//  Vector v = Vector2(0., 0.);
//  for (int i=1; i<=7; i++)
//    expected[ordering[X(i)]] = v;
//  EXPECT(assert_equal(expected,actual));
//}
//
///* ************************************************************************* */
//TEST( GaussianJunctionTreeB, optimizeMultiFrontal2)
//{
//  // create a graph
//  example::Graph nlfg = createNonlinearFactorGraph();
//  Values noisy = createNoisyValues();
//  Ordering ordering; ordering += X(1),X(2),L(1);
//  GaussianFactorGraph fg = *nlfg.linearize(noisy, ordering);
//
//  // optimize the graph
//  GaussianJunctionTree tree(fg);
//  VectorValues actual = tree.optimize(&EliminateQR);
//
//  // verify
//  VectorValues expected = createCorrectDelta(ordering); // expected solution
//  EXPECT(assert_equal(expected,actual));
//}
//
///* ************************************************************************* */
//TEST(GaussianJunctionTreeB, slamlike) {
//  Values init;
//  NonlinearFactorGraph newfactors;
//  NonlinearFactorGraph fullgraph;
//  SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, M_PI/100.0));
//  SharedDiagonal brNoise = noiseModel::Diagonal::Sigmas((Vector(2) << M_PI/100.0, 0.1));
//
//  size_t i = 0;
//
//  newfactors = NonlinearFactorGraph();
//  newfactors.add(PriorFactor<Pose2>(X(0), Pose2(0.0, 0.0, 0.0), odoNoise));
//  init.insert(X(0), Pose2(0.01, 0.01, 0.01));
//  fullgraph.push_back(newfactors);
//
//  for( ; i<5; ++i) {
//    newfactors = NonlinearFactorGraph();
//    newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
//    init.insert(X(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
//    fullgraph.push_back(newfactors);
//  }
//
//  newfactors = NonlinearFactorGraph();
//  newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
//  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(0), Rot2::fromAngle(M_PI/4.0), 5.0, brNoise));
//  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(1), Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise));
//  init.insert(X(i+1), Pose2(1.01, 0.01, 0.01));
//  init.insert(L(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
//  init.insert(L(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
//  fullgraph.push_back(newfactors);
//  ++ i;
//
//  for( ; i<5; ++i) {
//    newfactors = NonlinearFactorGraph();
//    newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
//    init.insert(X(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
//    fullgraph.push_back(newfactors);
//  }
//
//  newfactors = NonlinearFactorGraph();
//  newfactors.add(BetweenFactor<Pose2>(X(i), X(i+1), Pose2(1.0, 0.0, 0.0), odoNoise));
//  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(0), Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
//  newfactors.add(BearingRangeFactor<Pose2,Point2>(X(i), L(1), Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise));
//  init.insert(X(i+1), Pose2(6.9, 0.1, 0.01));
//  fullgraph.push_back(newfactors);
//  ++ i;
//
//  // Compare solutions
//  Ordering ordering = *fullgraph.orderingCOLAMD(init);
//  GaussianFactorGraph linearized = *fullgraph.linearize(init, ordering);
//
//  GaussianJunctionTree gjt(linearized);
//  VectorValues deltaactual = gjt.optimize(&EliminateQR);
//  Values actual = init.retract(deltaactual, ordering);
//
//  GaussianBayesNet gbn = *GaussianSequentialSolver(linearized).eliminate();
//  VectorValues delta = optimize(gbn);
//  Values expected = init.retract(delta, ordering);
//
//  EXPECT(assert_equal(expected, actual));
//}
//
///* ************************************************************************* */
//TEST(GaussianJunctionTreeB, simpleMarginal) {
//
//  typedef BayesTree<GaussianConditional> GaussianBayesTree;
//
//  // Create a simple graph
//  NonlinearFactorGraph fg;
//  fg.add(PriorFactor<Pose2>(X(0), Pose2(), noiseModel::Isotropic::Sigma(3, 10.0)));
//  fg.add(BetweenFactor<Pose2>(X(0), X(1), Pose2(1.0, 0.0, 0.0), noiseModel::Diagonal::Sigmas(Vector3(10.0, 1.0, 1.0))));
//
//  Values init;
//  init.insert(X(0), Pose2());
//  init.insert(X(1), Pose2(1.0, 0.0, 0.0));
//
//  Ordering ordering;
//  ordering += X(1), X(0);
//
//  GaussianFactorGraph gfg = *fg.linearize(init, ordering);
//
//  // Compute marginals with both sequential and multifrontal
//  Matrix expected = GaussianSequentialSolver(gfg).marginalCovariance(1);
//
//  Matrix actual1 = GaussianMultifrontalSolver(gfg).marginalCovariance(1);
//
//  // Compute marginal directly from marginal factor
//  GaussianFactor::shared_ptr marginalFactor = GaussianMultifrontalSolver(gfg).marginalFactor(1);
//  JacobianFactor::shared_ptr marginalJacobian = boost::dynamic_pointer_cast<JacobianFactor>(marginalFactor);
//  Matrix actual2 = inverse(marginalJacobian->getA(marginalJacobian->begin()).transpose() * marginalJacobian->getA(marginalJacobian->begin()));
//
//  // Compute marginal directly from BayesTree
//  GaussianBayesTree gbt;
//  gbt.insert(GaussianJunctionTree(gfg).eliminate(EliminateCholesky));
//  marginalFactor = gbt.marginalFactor(1, EliminateCholesky);
//  marginalJacobian = boost::dynamic_pointer_cast<JacobianFactor>(marginalFactor);
//  Matrix actual3 = inverse(marginalJacobian->getA(marginalJacobian->begin()).transpose() * marginalJacobian->getA(marginalJacobian->begin()));
//
//  EXPECT(assert_equal(expected, actual1));
//  EXPECT(assert_equal(expected, actual2));
//  EXPECT(assert_equal(expected, actual3));
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

