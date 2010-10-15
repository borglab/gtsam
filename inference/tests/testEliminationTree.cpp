/**
 * @file    testEliminationTree.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 14, 2010
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace gtsam;
using namespace std;

typedef EliminationTree<SymbolicFactorGraph> SymbolicEliminationTree;

struct EliminationTreeTester {
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
  Conditional::shared_ptr c0(new Conditional(0, 1, 2));
  Conditional::shared_ptr c1(new Conditional(1, 2, 4));
  Conditional::shared_ptr c2(new Conditional(2, 4));
  Conditional::shared_ptr c3(new Conditional(3, 4));
  Conditional::shared_ptr c4(new Conditional(4));

  SymbolicBayesNet expected;
  expected.push_back(c3);
  expected.push_back(c0);
  expected.push_back(c1);
  expected.push_back(c2);
  expected.push_back(c4);

  // Create factor graph
  SymbolicFactorGraph fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  SymbolicBayesNet actual = *SymbolicEliminationTree::Create(fg)->eliminate();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
