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
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/iterative.h>
#include <gtsam/linear/JacobianFactorGraph.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/inference/EliminationTree-inl.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* */
/** unnormalized error */
double error(const JacobianFactorGraph& fg, const VectorValues& x) {
  double total_error = 0.;
  BOOST_FOREACH(const JacobianFactor::shared_ptr& factor, fg)
    total_error += factor->error(x);
  return total_error;
}


/* ************************************************************************* */
TEST( SubgraphSolver, constructor1 )
{
  // Build a planar graph
  JacobianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = planarGraph(N); // A*x-b

  SubgraphSolverParameters parameters;
  SubgraphSolver solver(Ab, parameters);
  VectorValues optimized = solver.optimize();
  DOUBLES_EQUAL(0.0, error(Ab, optimized), 1e-5);
}

/* ************************************************************************* */
TEST( SubgraphSolver, constructor2 )
{
  // Build a planar graph
  JacobianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = planarGraph(N); // A*x-b

  // Get the spanning tree and corresponding ordering
  JacobianFactorGraph Ab1_, Ab2_; // A1*x-b1 and A2*x-b2
  boost::tie(Ab1_, Ab2_) = splitOffPlanarTree(N, Ab);

  SubgraphSolverParameters parameters;
  SubgraphSolver solver(Ab1_, Ab2_, parameters);
  VectorValues optimized = solver.optimize();
  DOUBLES_EQUAL(0.0, error(Ab, optimized), 1e-5);
}

/* ************************************************************************* */
TEST( SubgraphSolver, constructor3 )
{
  // Build a planar graph
  JacobianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = planarGraph(N); // A*x-b

  // Get the spanning tree and corresponding ordering
  JacobianFactorGraph Ab1_, Ab2_; // A1*x-b1 and A2*x-b2
  boost::tie(Ab1_, Ab2_) = splitOffPlanarTree(N, Ab);

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(Ab1_)->eliminate(&EliminateQR);

  SubgraphSolverParameters parameters;
  SubgraphSolver solver(Rc1, Ab2_, parameters);
  VectorValues optimized = solver.optimize();
  DOUBLES_EQUAL(0.0, error(Ab, optimized), 1e-5);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
