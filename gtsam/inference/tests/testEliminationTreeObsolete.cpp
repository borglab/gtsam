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

#include <gtsam/inference/EliminationTreeOrdered-inl.h>
#include <gtsam/inference/SymbolicSequentialSolverOrdered.h>

using namespace gtsam;
using namespace std;

class EliminationTreeOrderedTester {
public:
  // build hardcoded tree
  static SymbolicEliminationTreeOrdered::shared_ptr buildHardcodedTree(const SymbolicFactorGraphOrdered& fg) {

    SymbolicEliminationTreeOrdered::shared_ptr leaf0(new SymbolicEliminationTreeOrdered);
    leaf0->add(fg[0]);
    leaf0->add(fg[1]);

    SymbolicEliminationTreeOrdered::shared_ptr node1(new SymbolicEliminationTreeOrdered(1));
    node1->add(fg[2]);
    node1->add(leaf0);

    SymbolicEliminationTreeOrdered::shared_ptr node2(new SymbolicEliminationTreeOrdered(2));
    node2->add(fg[3]);
    node2->add(node1);

    SymbolicEliminationTreeOrdered::shared_ptr leaf3(new SymbolicEliminationTreeOrdered(3));
    leaf3->add(fg[4]);

    SymbolicEliminationTreeOrdered::shared_ptr etree(new SymbolicEliminationTreeOrdered(4));
    etree->add(leaf3);
    etree->add(node2);

    return etree;
  }
};

TEST(EliminationTreeOrdered, Create)
{
  // create example factor graph
  SymbolicFactorGraphOrdered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  SymbolicEliminationTreeOrdered::shared_ptr expected = EliminationTreeOrderedTester::buildHardcodedTree(fg);

  // Build from factor graph
  SymbolicEliminationTreeOrdered::shared_ptr actual = SymbolicEliminationTreeOrdered::Create(fg);

  CHECK(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
// Test to drive elimination tree development
// graph: f(0,1) f(0,2) f(1,4) f(2,4) f(3,4)
/* ************************************************************************* */

TEST(EliminationTreeOrdered, eliminate )
{
  // create expected Chordal bayes Net
  SymbolicBayesNetOrdered expected;
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(3,4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(2,4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(1,2,4));
  expected.push_front(boost::make_shared<IndexConditionalOrdered>(0,1,2));

  // Create factor graph
  SymbolicFactorGraphOrdered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  SymbolicBayesNetOrdered actual = *SymbolicSequentialSolver(fg).eliminate();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(EliminationTreeOrdered, disconnected_graph) {
  SymbolicFactorGraphOrdered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 2);
  fg.push_factor(3, 4);

  CHECK_EXCEPTION(SymbolicEliminationTreeOrdered::Create(fg), DisconnectedGraphException);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
