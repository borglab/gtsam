/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicBayesNetB.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <tests/smallExample.h>
#include <gtsam/inference/SymbolicFactorGraphOrdered.h>
#include <gtsam/inference/SymbolicSequentialSolverOrdered.h>
#include <gtsam/nonlinear/OrderingOrdered.h>
#include <gtsam/nonlinear/Symbol.h>

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( SymbolicBayesNetOrdered, constructor )
{
  OrderingOrdered o; o += X(2),L(1),X(1);
  // Create manually
  IndexConditionalOrdered::shared_ptr
    x2(new IndexConditionalOrdered(o[X(2)],o[L(1)], o[X(1)])),
    l1(new IndexConditionalOrdered(o[L(1)],o[X(1)])),
    x1(new IndexConditionalOrdered(o[X(1)]));
  BayesNetOrdered<IndexConditionalOrdered> expected;
  expected.push_back(x2);
  expected.push_back(l1);
  expected.push_back(x1);

  // Create from a factor graph
  GaussianFactorGraphOrdered factorGraph = createGaussianFactorGraph(o);
  SymbolicFactorGraphOrdered fg(factorGraph);

  // eliminate it
  SymbolicBayesNetOrdered actual = *SymbolicSequentialSolver(fg).eliminate();

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( SymbolicBayesNetOrdered, FromGaussian) {
  SymbolicBayesNetOrdered expected;
  expected.push_back(IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(0, 1)));
  expected.push_back(IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(1)));

  GaussianBayesNetOrdered gbn = createSmallGaussianBayesNet();
  SymbolicBayesNetOrdered actual(gbn);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
