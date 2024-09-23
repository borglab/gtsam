/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicEliminationTree.cpp
 * @brief
 * @author  Richard Roberts
 * @date Oct 14, 2010
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>

#include <vector>

#include "symbolicExampleGraphs.h"

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace std;
using sharedNode = SymbolicEliminationTree::sharedNode;

// Use list_of replacement defined in symbolicExampleGraphs.h
using ChildNodes = ChainedVector<sharedNode>;

class EliminationTreeTester {
 public:
  // build hardcoded tree
  static SymbolicEliminationTree buildHardcodedTree(
      const SymbolicFactorGraph& fg) {
    sharedNode leaf0(new SymbolicEliminationTree::Node);
    leaf0->key = 0;
    leaf0->factors.push_back(fg[0]);
    leaf0->factors.push_back(fg[1]);

    sharedNode node1(new SymbolicEliminationTree::Node);
    node1->key = 1;
    node1->factors.push_back(fg[2]);
    node1->children.push_back(leaf0);

    sharedNode node2(new SymbolicEliminationTree::Node);
    node2->key = 2;
    node2->factors.push_back(fg[3]);
    node2->children.push_back(node1);

    sharedNode leaf3(new SymbolicEliminationTree::Node);
    leaf3->key = 3;
    leaf3->factors.push_back(fg[4]);

    sharedNode root(new SymbolicEliminationTree::Node);
    root->key = 4;
    root->children.push_back(leaf3);
    root->children.push_back(node2);

    SymbolicEliminationTree tree;
    tree.roots_.push_back(root);
    return tree;
  }

  static SymbolicEliminationTree MakeTree(const ChildNodes::Result& roots) {
    SymbolicEliminationTree et;
    et.roots_.assign(roots.begin(), roots.end());
    return et;
  }
};

// Create a leaf node.
static sharedNode Leaf(Key key, const SymbolicFactorGraph& factors) {
  sharedNode node(new SymbolicEliminationTree::Node());
  node->key = key;
  node->factors.assign(factors.begin(), factors.end());
  return node;
}

// Create a node with children.
static sharedNode Node(Key key, const SymbolicFactorGraph& factors,
                       const ChildNodes::Result& children) {
  sharedNode node = Leaf(key, factors);
  node->children.assign(children.begin(), children.end());
  return node;
}

/* ************************************************************************* */
TEST(EliminationTree, Create) {
  SymbolicEliminationTree expected =
      EliminationTreeTester::buildHardcodedTree(simpleTestGraph1);

  // Build from factor graph
  const Ordering order{0, 1, 2, 3, 4};
  SymbolicEliminationTree actual(simpleTestGraph1, order);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(EliminationTree, Create2) {
  //        l1                  l2
  //    /    |                /  |
  // x1 --- x2 --- x3 --- x4 --- x5
  //                          \  |
  //                            l3
  SymbolicFactorGraph graph;
  graph.emplace_shared<SymbolicFactor>(X(1), L(1));
  graph.emplace_shared<SymbolicFactor>(X(1), X(2));
  graph.emplace_shared<SymbolicFactor>(X(2), L(1));
  graph.emplace_shared<SymbolicFactor>(X(2), X(3));
  graph.emplace_shared<SymbolicFactor>(X(3), X(4));
  graph.emplace_shared<SymbolicFactor>(X(4), L(2));
  graph.emplace_shared<SymbolicFactor>(X(4), X(5));
  graph.emplace_shared<SymbolicFactor>(L(2), X(5));
  graph.emplace_shared<SymbolicFactor>(X(4), L(3));
  graph.emplace_shared<SymbolicFactor>(X(5), L(3));

  auto binary = [](Key j1, Key j2) -> SymbolicFactor {
    return SymbolicFactor(j1, j2);
  };

  SymbolicEliminationTree expected = EliminationTreeTester::MakeTree(  //
      ChildNodes(                                                      //
          Node(X(3), SymbolicFactorGraph(),
               ChildNodes(  //
                   Node(X(2), SymbolicFactorGraph(binary(X(2), X(3))),
                        ChildNodes(  //
                            Node(L(1), SymbolicFactorGraph(binary(X(2), L(1))),
                                 ChildNodes(  //
                                     Leaf(X(1), SymbolicFactorGraph(
                                                    binary(X(1), L(1)))(
                                                    binary(X(1), X(2)))))))))(
                   Node(X(4), SymbolicFactorGraph(binary(X(3), X(4))),
                        ChildNodes(  //
                            Node(L(2), SymbolicFactorGraph(binary(X(4), L(2))),
                                 ChildNodes(  //
                                     Node(X(5),
                                          SymbolicFactorGraph(binary(
                                              X(4), X(5)))(binary(L(2), X(5))),
                                          ChildNodes(  //
                                              Leaf(L(3),
                                                   SymbolicFactorGraph(
                                                       binary(X(4), L(3)))(
                                                       binary(X(5), L(3)))  //
                                                   )                        //
                                              )                             //
                                          )                                 //
                                     )                                      //
                                 )                                          //
                            )                                               //
                        )                                                   //
                   )                                                        //
               )                                                            //
          )                                                                 //
  );

  const Ordering order{X(1), L(3), L(1), X(5), X(2), L(2), X(4), X(3)};
  SymbolicEliminationTree actual(graph, order);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
