/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file testLeveledSpanningTree
 * @brief Unit tests for finding fundamental cycles in a graph.
 * @author Frank Dellaert
 */

#include <gtsam/base/LeveledSpanningTree.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <utility>


using namespace gtsam;


/* ************************************************************************* */
TEST(FundamentalCycles, AdjacencyList) {
    // Test using adjacency list
    LeveledSpanningTree::Graph graph = {
            {1, 2},    // neighbors of vertex 0
            {0, 2, 3}, // neighbors of vertex 1
            {0, 1, 3}, // neighbors of vertex 2
            {1, 2, 4}, // neighbors of vertex 3
            {3}        // neighbors of vertex 4
    };

    const LeveledSpanningTree lst(graph, 0);
    auto result = lst.allFundamentalCycles(graph);

    // Verify the number of fundamental cycles
    EXPECT(result.size() == 2);
}

/* ************************************************************************* */
TEST(FundamentalCycles, EdgeList) {
    // Test using edge list
    struct Edge {
        size_t i;
        size_t j;
    };

    std::vector<Edge> edges = {
        {0, 1},
        {0, 2},
        {1, 2},
        {1, 3},
        {2, 3},
        {3, 4}
    };

    const LeveledSpanningTree lst = LeveledSpanningTree::FromEdges(edges, 0);
    auto result = lst.allFundamentalCycles(edges);

    // Verify the number of fundamental cycles
    EXPECT(result.size() == 2);
    for (const auto& cycle : result) {
        size_t i = cycle.front();
        size_t j = cycle.back();
        EXPECT(!lst.contains(i, j));
    }
}

/* ************************************************************************* */
TEST(FundamentalCycles, SymbolicFactorGraph) {
    using symbol_shorthand::X;

    // Create factor graph.
    SymbolicFactorGraph g;
    g.push_factor(X(1), X(2));
    g.push_factor(X(1), X(3));
    g.push_factor(X(1), X(4));
    g.push_factor(X(2), X(3));
    g.push_factor(X(2), X(4));
    g.push_factor(X(3), X(4));

    std::map<Key, size_t> keyToIndex;
    std::vector<size_t> indexToKey;
    auto lst = LeveledSpanningTree::FromFactorGraph(g, X(1), &keyToIndex, &indexToKey);
    auto result = lst.allFundamentalCycles(g, keyToIndex, indexToKey);

    // Verify the number of fundamental cycles
    EXPECT(result.size() == 3);
}


/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
