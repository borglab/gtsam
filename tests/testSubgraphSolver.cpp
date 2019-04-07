/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testSubgraphSolver.cpp
 *  @brief  Unit tests for SubgraphSolver
 *  @author Yong-Dian Jian
 **/

#include <tests/smallExample.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/iterative.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

static size_t N = 3;
static SubgraphSolverParameters kParameters;
static auto kOrdering = example::planarOrdering(N);

/* ************************************************************************* */
/** unnormalized error */
static double error(const GaussianFactorGraph& fg, const VectorValues& x) {
  double total_error = 0.;
  for(const GaussianFactor::shared_ptr& factor: fg)
    total_error += factor->error(x);
  return total_error;
}

/* ************************************************************************* */
TEST( SubgraphSolver, Parameters )
{
  LONGS_EQUAL(SubgraphSolverParameters::SILENT, kParameters.verbosity());
  LONGS_EQUAL(500, kParameters.maxIterations());
}

/* ************************************************************************* */
TEST( SubgraphSolver, constructor1 )
{
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  boost::tie(Ab, xtrue) = example::planarGraph(N); // A*x-b

  // The first constructor just takes a factor graph (and kParameters)
  // and it will split the graph into A1 and A2, where A1 is a spanning tree
  SubgraphSolver solver(Ab, kParameters, kOrdering);
  VectorValues optimized = solver.optimize(); // does PCG optimization
  DOUBLES_EQUAL(0.0, error(Ab, optimized), 1e-5);
}

/* ************************************************************************* */
TEST( SubgraphSolver, constructor2 )
{
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = example::planarGraph(N); // A*x-b

  // Get the spanning tree
  GaussianFactorGraph::shared_ptr Ab1, Ab2; // A1*x-b1 and A2*x-b2
  boost::tie(Ab1, Ab2) = example::splitOffPlanarTree(N, Ab);

  // The second constructor takes two factor graphs, so the caller can specify
  // the preconditioner (Ab1) and the constraints that are left out (Ab2)
  SubgraphSolver solver(*Ab1, Ab2, kParameters, kOrdering);
  VectorValues optimized = solver.optimize();
  DOUBLES_EQUAL(0.0, error(Ab, optimized), 1e-5);
}

/* ************************************************************************* */
TEST( SubgraphSolver, constructor3 )
{
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = example::planarGraph(N); // A*x-b

  // Get the spanning tree and corresponding kOrdering
  GaussianFactorGraph::shared_ptr Ab1, Ab2; // A1*x-b1 and A2*x-b2
  boost::tie(Ab1, Ab2) = example::splitOffPlanarTree(N, Ab);

  // The caller solves |A1*x-b1|^2 == |R1*x-c1|^2, where R1 is square UT
  auto Rc1 = Ab1->eliminateSequential();

  // The third constructor allows the caller to pass an already solved preconditioner Rc1_
  // as a Bayes net, in addition to the "loop closing constraints" Ab2, as before
  SubgraphSolver solver(Rc1, Ab2, kParameters);
  VectorValues optimized = solver.optimize();
  DOUBLES_EQUAL(0.0, error(Ab, optimized), 1e-5);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
