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

#include <gtsam/inference/SymbolicSequentialSolver.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

TEST( SymbolicSequentialSolver, SymbolicSequentialSolver ) {
  // create factor graph
  SymbolicFactorGraph g;
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
  SymbolicFactorGraph fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  SymbolicSequentialSolver solver(fg);
  SymbolicBayesNet::shared_ptr actual = solver.eliminate();
  SymbolicBayesNet expected;
  expected.push_front(boost::make_shared<IndexConditional>(4));
  expected.push_front(boost::make_shared<IndexConditional>(3, 4));
  expected.push_front(boost::make_shared<IndexConditional>(2, 4));
  expected.push_front(boost::make_shared<IndexConditional>(1, 2, 4));
  expected.push_front(boost::make_shared<IndexConditional>(0, 1, 2));
  EXPECT(assert_equal(expected,*actual));

  {
    // jointBayesNet
    vector<Index> js;
    js.push_back(0);
    js.push_back(4);
    js.push_back(3);
    SymbolicBayesNet::shared_ptr actualBN = solver.jointBayesNet(js);
    SymbolicBayesNet expectedBN;
    expectedBN.push_front(boost::make_shared<IndexConditional>(3));
    expectedBN.push_front(boost::make_shared<IndexConditional>(4, 3));
    expectedBN.push_front(boost::make_shared<IndexConditional>(0, 4));
    EXPECT( assert_equal(expectedBN,*actualBN));

    // jointFactorGraph
    SymbolicFactorGraph::shared_ptr actualFG = solver.jointFactorGraph(js);
    SymbolicFactorGraph expectedFG;
    expectedFG.push_factor(0, 4);
    expectedFG.push_factor(4, 3);
    expectedFG.push_factor(3);
    EXPECT( assert_equal(expectedFG,(SymbolicFactorGraph)(*actualFG)));
  }

  {
    // jointBayesNet
    vector<Index> js;
    js.push_back(0);
    js.push_back(2);
    js.push_back(3);
    SymbolicBayesNet::shared_ptr actualBN = solver.jointBayesNet(js);
    SymbolicBayesNet expectedBN;
    expectedBN.push_front(boost::make_shared<IndexConditional>(2));
    expectedBN.push_front(boost::make_shared<IndexConditional>(3, 2));
    expectedBN.push_front(boost::make_shared<IndexConditional>(0, 3, 2));
    EXPECT( assert_equal(expectedBN,*actualBN));

    // jointFactorGraph
    SymbolicFactorGraph::shared_ptr actualFG = solver.jointFactorGraph(js);
    SymbolicFactorGraph expectedFG;
    expectedFG.push_factor(0, 3, 2);
    expectedFG.push_factor(3, 2);
    expectedFG.push_factor(2);
    EXPECT( assert_equal(expectedFG,(SymbolicFactorGraph)(*actualFG)));
  }

  {
    // conditionalBayesNet
    vector<Index> js;
    js.push_back(0);
    js.push_back(2);
    js.push_back(3);
    size_t nrFrontals = 2;
    SymbolicBayesNet::shared_ptr actualBN = //
        solver.conditionalBayesNet(js, nrFrontals);
    SymbolicBayesNet expectedBN;
    expectedBN.push_front(boost::make_shared<IndexConditional>(2, 3));
    expectedBN.push_front(boost::make_shared<IndexConditional>(0, 2, 3));
    EXPECT( assert_equal(expectedBN,*actualBN));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
