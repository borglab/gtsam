/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testEliminationTree.cpp
 * @brief   
 * @author  Richard Roberts
 * @date Oct 14, 2010
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/EliminationTree-inl.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>

using namespace gtsam;
using namespace std;

class EliminationTreeTester {
public:
  // build hardcoded tree
  static SymbolicEliminationTree::shared_ptr buildHardcodedTree(const SymbolicFactorGraph& fg) {

    SymbolicEliminationTree::shared_ptr leaf0(new SymbolicEliminationTree);
    leaf0->add(fg[0]);
    leaf0->add(fg[1]);

    SymbolicEliminationTree::shared_ptr node1(new SymbolicEliminationTree(1));
    node1->add(fg[2]);
    node1->add(leaf0);

    SymbolicEliminationTree::shared_ptr node2(new SymbolicEliminationTree(2));
    node2->add(fg[3]);
    node2->add(node1);

    SymbolicEliminationTree::shared_ptr leaf3(new SymbolicEliminationTree(3));
    leaf3->add(fg[4]);

    SymbolicEliminationTree::shared_ptr etree(new SymbolicEliminationTree(4));
    etree->add(leaf3);
    etree->add(node2);

    return etree;
  }
};

TEST(EliminationTree, Create)
{
  // create example factor graph
  SymbolicFactorGraph fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  SymbolicEliminationTree::shared_ptr expected = EliminationTreeTester::buildHardcodedTree(fg);

  // Build from factor graph
  SymbolicEliminationTree::shared_ptr actual = SymbolicEliminationTree::Create(fg);

  CHECK(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
// Test to drive elimination tree development
// graph: f(0,1) f(0,2) f(1,4) f(2,4) f(3,4)
/* ************************************************************************* */

TEST(EliminationTree, eliminate )
{
  // create expected Chordal bayes Net
  SymbolicBayesNet expected;
  expected.push_front(boost::make_shared<IndexConditional>(4));
  expected.push_front(boost::make_shared<IndexConditional>(3,4));
  expected.push_front(boost::make_shared<IndexConditional>(2,4));
  expected.push_front(boost::make_shared<IndexConditional>(1,2,4));
  expected.push_front(boost::make_shared<IndexConditional>(0,1,2));

  // Create factor graph
  SymbolicFactorGraph fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  SymbolicBayesNet actual = *SymbolicSequentialSolver(fg).eliminate();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(EliminationTree, disconnected_graph) {
  SymbolicFactorGraph fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 2);
  fg.push_factor(3, 4);

  CHECK_EXCEPTION(SymbolicEliminationTree::Create(fg), DisconnectedGraphException);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
