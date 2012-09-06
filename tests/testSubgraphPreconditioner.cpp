/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testSubgraphConditioner.cpp
 *  @brief  Unit tests for SubgraphPreconditioner
 *  @author Frank Dellaert
 **/

#include <tests/smallExample.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/iterative.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;
using namespace example;

// define keys
// Create key for simulated planar graph
Symbol key(int x, int y) {
  return symbol_shorthand::X(1000*x+y);
}

/* ************************************************************************* */
TEST( SubgraphPreconditioner, planarOrdering ) {
  // Check canonical ordering
  Ordering expected, ordering = planarOrdering(3);
  expected +=
      key(3, 3), key(2, 3), key(1, 3),
      key(3, 2), key(2, 2), key(1, 2),
      key(3, 1), key(2, 1), key(1, 1);
  CHECK(assert_equal(expected,ordering));
}

/* ************************************************************************* */
/** unnormalized error */
static double error(const GaussianFactorGraph& fg, const VectorValues& x) {
  double total_error = 0.;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, fg)
    total_error += factor->error(x);
  return total_error;
}

/* ************************************************************************* */
TEST( SubgraphPreconditioner, planarGraph )
  {
  // Check planar graph construction
  GaussianFactorGraph A;
  VectorValues xtrue;
  boost::tie(A, xtrue) = planarGraph(3);
  LONGS_EQUAL(13,A.size());
  LONGS_EQUAL(9,xtrue.size());
  DOUBLES_EQUAL(0,error(A,xtrue),1e-9); // check zero error for xtrue

  // Check that xtrue is optimal
  GaussianBayesNet::shared_ptr R1 = GaussianSequentialSolver(A).eliminate();
  VectorValues actual = optimize(*R1);
  CHECK(assert_equal(xtrue,actual));
}

/* ************************************************************************* */
TEST( SubgraphPreconditioner, splitOffPlanarTree )
{
  // Build a planar graph
  GaussianFactorGraph A;
  VectorValues xtrue;
  boost::tie(A, xtrue) = planarGraph(3);

  // Get the spanning tree and constraints, and check their sizes
  GaussianFactorGraph T, C;
  boost::tie(T, C) = splitOffPlanarTree(3, A);
  LONGS_EQUAL(9,T.size());
  LONGS_EQUAL(4,C.size());

  // Check that the tree can be solved to give the ground xtrue
  GaussianBayesNet::shared_ptr R1 = GaussianSequentialSolver(T).eliminate();
  VectorValues xbar = optimize(*R1);
  CHECK(assert_equal(xtrue,xbar));
}

/* ************************************************************************* */

TEST( SubgraphPreconditioner, system )
{
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = planarGraph(N); // A*x-b

  // Get the spanning tree and corresponding ordering
  GaussianFactorGraph Ab1_, Ab2_; // A1*x-b1 and A2*x-b2
  boost::tie(Ab1_, Ab2_) = splitOffPlanarTree(N, Ab);
  SubgraphPreconditioner::sharedFG Ab1(new GaussianFactorGraph(Ab1_));
  SubgraphPreconditioner::sharedFG Ab2(new GaussianFactorGraph(Ab2_));

  // Eliminate the spanning tree to build a prior
  SubgraphPreconditioner::sharedBayesNet Rc1 = GaussianSequentialSolver(Ab1_).eliminate(); // R1*x-c1
  VectorValues xbar = optimize(*Rc1); // xbar = inv(R1)*c1

  // Create Subgraph-preconditioned system
  VectorValues::shared_ptr xbarShared(new VectorValues(xbar)); // TODO: horrible
  SubgraphPreconditioner system(Ab2, Rc1, xbarShared);

  // Create zero config
  VectorValues zeros = VectorValues::Zero(xbar);

  // Set up y0 as all zeros
  VectorValues y0 = zeros;

  // y1 = perturbed y0
  VectorValues y1 = zeros;
  y1[1] = Vector_(2, 1.0, -1.0);

  // Check corresponding x  values
  VectorValues expected_x1 = xtrue, x1 = system.x(y1);
  expected_x1[1] = Vector_(2, 2.01, 2.99);
  expected_x1[0] = Vector_(2, 3.01, 2.99);
  CHECK(assert_equal(xtrue, system.x(y0)));
  CHECK(assert_equal(expected_x1,system.x(y1)));

  // Check errors
  DOUBLES_EQUAL(0,error(Ab,xtrue),1e-9);
  DOUBLES_EQUAL(3,error(Ab,x1),1e-9);
  DOUBLES_EQUAL(0,error(system,y0),1e-9);
  DOUBLES_EQUAL(3,error(system,y1),1e-9);

  // Test gradient in x
  VectorValues expected_gx0 = zeros;
  VectorValues expected_gx1 = zeros;
  CHECK(assert_equal(expected_gx0,gradient(Ab,xtrue)));
  expected_gx1[2] = Vector_(2, -100., 100.);
  expected_gx1[4] = Vector_(2, -100., 100.);
  expected_gx1[1] = Vector_(2, 200., -200.);
  expected_gx1[3] = Vector_(2, -100., 100.);
  expected_gx1[0] = Vector_(2, 100., -100.);
  CHECK(assert_equal(expected_gx1,gradient(Ab,x1)));

  // Test gradient in y
  VectorValues expected_gy0 = zeros;
  VectorValues expected_gy1 = zeros;
  expected_gy1[2] = Vector_(2, 2., -2.);
  expected_gy1[4] = Vector_(2, -2., 2.);
  expected_gy1[1] = Vector_(2, 3., -3.);
  expected_gy1[3] = Vector_(2, -1., 1.);
  expected_gy1[0] = Vector_(2, 1., -1.);
  CHECK(assert_equal(expected_gy0,gradient(system,y0)));
  CHECK(assert_equal(expected_gy1,gradient(system,y1)));

  // Check it numerically for good measure
  // TODO use boost::bind(&SubgraphPreconditioner::error,&system,_1)
  //	Vector numerical_g1 = numericalGradient<VectorValues> (error, y1, 0.001);
  //	Vector expected_g1 = Vector_(18, 0., 0., 0., 0., 2., -2., 0., 0., -2., 2.,
  //			3., -3., 0., 0., -1., 1., 1., -1.);
  //	CHECK(assert_equal(expected_g1,numerical_g1));
}

/* ************************************************************************* */
TEST( SubgraphPreconditioner, conjugateGradients )
{
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = planarGraph(N); // A*x-b

  // Get the spanning tree and corresponding ordering
  GaussianFactorGraph Ab1_, Ab2_; // A1*x-b1 and A2*x-b2
  boost::tie(Ab1_, Ab2_) = splitOffPlanarTree(N, Ab);
  SubgraphPreconditioner::sharedFG Ab1(new GaussianFactorGraph(Ab1_));
  SubgraphPreconditioner::sharedFG Ab2(new GaussianFactorGraph(Ab2_));

  // Eliminate the spanning tree to build a prior
  Ordering ordering = planarOrdering(N);
  SubgraphPreconditioner::sharedBayesNet Rc1 = GaussianSequentialSolver(Ab1_).eliminate(); // R1*x-c1
  VectorValues xbar = optimize(*Rc1); // xbar = inv(R1)*c1

  // Create Subgraph-preconditioned system
  VectorValues::shared_ptr xbarShared(new VectorValues(xbar)); // TODO: horrible
  SubgraphPreconditioner system(Ab2, Rc1, xbarShared);

  // Create zero config y0 and perturbed config y1
  VectorValues y0 = VectorValues::Zero(xbar);

  VectorValues y1 = y0;
  y1[1] = Vector_(2, 1.0, -1.0);
  VectorValues x1 = system.x(y1);

  // Solve for the remaining constraints using PCG
  ConjugateGradientParameters parameters;
  VectorValues actual = conjugateGradients<SubgraphPreconditioner,
      VectorValues, Errors>(system, y1, parameters);
  CHECK(assert_equal(y0,actual));

  // Compare with non preconditioned version:
  VectorValues actual2 = conjugateGradientDescent(Ab, x1, parameters);
  CHECK(assert_equal(xtrue,actual2,1e-4));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
