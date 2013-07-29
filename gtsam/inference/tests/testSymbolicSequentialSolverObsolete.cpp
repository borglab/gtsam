/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicSequentialSolver.cpp
 * @brief   Unit tests for a symbolic sequential solver routines
 * @author  Frank Dellaert
 * @date    Sept 16, 2012
 */

#include <gtsam/inference/SymbolicSequentialSolverOrdered.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

TEST( SymbolicSequentialSolver, SymbolicSequentialSolver ) {
  // create factor graph
  SymbolicFactorGraphOrdered g;
  g.push_factor(2, 1, 0);
  g.push_factor(2, 0);
  g.push_factor(2);
  // test solver is Testable
  SymbolicSequentialSolver solver(g);
//  GTSAM_PRINT(solver);
  EXPECT(assert_equal(solver,solver));
}

/* ************************************************************************* */

TEST( SymbolicSequentialSolver, inference ) {
  // Create factor graph
  SymbolicFactorGraphOrdered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  SymbolicSequentialSolver solver(fg);
  SymbolicBayesNetOrdered::shared_ptr actual = solver.eliminate();
  SymbolicBayesNetOrdered expected;
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(3, 4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(2, 4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(1, 2, 4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(0, 1, 2));
  EXPECT(assert_equal(expected,*actual));

  {
    // jointBayesNet
    vector<Index> js;
    js.push_back(0);
    js.push_back(4);
    js.push_back(3);
    SymbolicBayesNetOrdered::shared_ptr actualBN = solver.jointBayesNet(js);
    SymbolicBayesNetOrdered expectedBN;
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(3));
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(4, 3));
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(0, 4));
    EXPECT( assert_equal(expectedBN,*actualBN));

    // jointFactorGraph
    SymbolicFactorGraphOrdered::shared_ptr actualFG = solver.jointFactorGraph(js);
    SymbolicFactorGraphOrdered expectedFG;
    expectedFG.push_factor(0, 4);
    expectedFG.push_factor(4, 3);
    expectedFG.push_factor(3);
    EXPECT( assert_equal(expectedFG,(SymbolicFactorGraphOrdered)(*actualFG)));
  }

  {
    // jointBayesNet
    vector<Index> js;
    js.push_back(0);
    js.push_back(2);
    js.push_back(3);
    SymbolicBayesNetOrdered::shared_ptr actualBN = solver.jointBayesNet(js);
    SymbolicBayesNetOrdered expectedBN;
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(2));
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(3, 2));
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(0, 3, 2));
    EXPECT( assert_equal(expectedBN,*actualBN));

    // jointFactorGraph
    SymbolicFactorGraphOrdered::shared_ptr actualFG = solver.jointFactorGraph(js);
    SymbolicFactorGraphOrdered expectedFG;
    expectedFG.push_factor(0, 3, 2);
    expectedFG.push_factor(3, 2);
    expectedFG.push_factor(2);
    EXPECT( assert_equal(expectedFG,(SymbolicFactorGraphOrdered)(*actualFG)));
  }

  {
    // conditionalBayesNet
    vector<Index> js;
    js.push_back(0);
    js.push_back(2);
    js.push_back(3);
    size_t nrFrontals = 2;
    SymbolicBayesNetOrdered::shared_ptr actualBN = //
        solver.conditionalBayesNet(js, nrFrontals);
    SymbolicBayesNetOrdered expectedBN;
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(2, 3));
    expectedBN.push_front(boost::make_shared<IndexConditionalOrdered>(0, 2, 3));
    EXPECT( assert_equal(expectedBN,*actualBN));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
