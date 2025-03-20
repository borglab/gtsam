/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file testFindCycles
 * @brief Unit tests for finding fundamental cycles in a graph.
 * @author Frank Dellaert
 */

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



// FundamentalCycle: Wraps a cycle (sequence of vertex indices). (O(1))
struct FundamentalCycle {
    std::vector<size_t> cycle;
};

// LeveledBfsTree: Holds BFS tree information.
struct LeveledBfsTree {
    struct Node {
        size_t parent;
        size_t level;
    };

    std::vector<std::optional<Node>> nodes; // nodes[v] = {parent, level} for vertex v

    LeveledBfsTree(size_t n, size_t root) : nodes(n, std::nullopt) {
        // Root has no parent, so we use itself as parent
        setNode(root, root, 0);
    }

    void setNode(size_t vertex, size_t parent, size_t level) {
        nodes[vertex] = Node{ parent, level };
    }

    const std::optional<Node>& operator[](size_t vertex) const {
        return nodes[vertex];
    }

    size_t level(size_t vertex) const {
        return nodes[vertex]->level;
    }

    size_t parent(size_t vertex) const {
        return nodes[vertex]->parent;
    }

    /**
     * computeFundamentalCycle: Computes the fundamental cycle corresponding to a non-tree edge (u,v).
     * Worst-case O(V) if the tree depth is O(V).
     */
    FundamentalCycle computeFundamentalCycle(size_t u, size_t v) {
        std::vector<size_t> pathU, pathV;
        // Climb up from u until levels are equal.
        while (level(u) > level(v)) {
            pathU.push_back(u);
            u = parent(u);
        }
        // Climb up from v until levels are equal.
        while (level(v) > level(u)) {
            pathV.push_back(v);
            v = parent(v);
        }
        // Climb until the lowest common ancestor is reached.
        while (u != v) {
            pathU.push_back(u);
            pathV.push_back(v);
            u = parent(u);
            v = parent(v);
        }
        pathU.push_back(u); // Add the LCA.
        // Append the reverse of pathV to complete the cycle.
        pathU.insert(pathU.end(), pathV.rbegin(), pathV.rend());
        return { pathU };
    }

};

// GraphList: adjacency list representation (vector of vectors of size_t)
using GraphList = std::vector<std::vector<size_t>>;

/** bfsTree: Performs a breadth-first search on the given adjacency list starting at root. O(V+E) */
LeveledBfsTree bfsTree(const GraphList& graph, size_t root) {
    LeveledBfsTree result(graph.size(), root);

    std::queue<size_t> queue;
    queue.push(root);

    while (!queue.empty()) {
        size_t current = queue.front();
        queue.pop();

        const auto& currentNode = result.nodes[current];
        size_t currentLevel = currentNode->level;

        for (size_t neighbor : graph[current]) {
            if (!result[neighbor]) {
                result.setNode(neighbor, current, currentLevel + 1);
                queue.push(neighbor);
            }
        }
    }
    return result;
}

// FundamentalCyclesResult: Holds all computed fundamental cycles and the tree edges
struct FundamentalCyclesResult {
    std::vector<FundamentalCycle> cycles;
    std::set<std::pair<size_t, size_t>> tree; // Tree edges as a set
};

/**
 * computeFundamentalCycles:
 * Given an adjacency list, computes all fundamental cycles using a BFS tree.
 * Running time: O(V+E) for the BFS plus up to O(E*V) in the worst-case for cycle computation.
 */
FundamentalCyclesResult computeFundamentalCycles(const GraphList& graph, size_t root) {
    size_t n = graph.size();
    auto bfsResult = bfsTree(graph, root);

    // Initialize result
    FundamentalCyclesResult result;

    // Collect tree edges in O(V)
    for (size_t v = 0; v < n; ++v) {
        auto node = bfsResult[v];
        if (node->parent != v) {
            size_t a = std::min(v, node->parent);
            size_t b = std::max(v, node->parent);
            result.tree.emplace(a, b);
        }
    }

    // Compute fundamental cycles for each non-tree edge in O(E) iterations (each worst-case O(V))
    for (size_t u = 0; u < n; ++u) {
        for (size_t v : graph[u]) {
            if (u < v && result.tree.find({ u, v }) == result.tree.end()) {
                result.cycles.push_back(bfsResult.computeFundamentalCycle(u, v));
            }
        }
    }

    return result;
}

/** buildAdjacencyList: Converts an edge list to an adjacency list. O(E) */
template <typename EdgeType>
GraphList buildAdjacencyList(const std::vector<EdgeType>& edges, size_t numNodes) {
    GraphList graph(numNodes);
    for (const auto& edge : edges) {
        // Assuming an undirected graph.
        graph[edge.i].push_back(static_cast<size_t>(edge.j));
        graph[edge.j].push_back(static_cast<size_t>(edge.i));
    }
    return graph;
}

/**
 * computeFundamentalCycles:
 * Given an edge list, converts it to an adjacency list (O(E)) and then computes fundamental cycles.
 * Overall running time: O(E + (Adjacency list version)).
 */
template <typename EdgeType>
FundamentalCyclesResult computeFundamentalCycles(const std::vector<EdgeType>& edges, size_t numNodes, size_t root) {
    GraphList graph = buildAdjacencyList(edges, numNodes); // O(E)
    return computeFundamentalCycles(graph, root);   // See function comment above.
}
/**
 * buildAdjacencyListFromFactors: Builds an adjacency list from binary factors in a graph. O(F)
 */
template <typename FactorGraph>
GraphList buildAdjacencyListFromFactors(const FactorGraph& graph,
    std::map<Key, size_t>* keyToIndex,
    std::vector<size_t>* indexToKey) {
    keyToIndex->clear();
    indexToKey->clear();
    GraphList adjacencyList;

    // Process all binary factors in a single pass
    for (const auto& factor : graph) {
        if (!factor || factor->keys().size() != 2) continue;

        const auto& keys = factor->keys();

        // Get or create indices for both keys
        for (const auto& key : keys) {
            if (keyToIndex->find(key) == keyToIndex->end()) {
                const size_t n = keyToIndex->size();
                keyToIndex->emplace(key, n);
                indexToKey->push_back(key);
                adjacencyList.push_back({}); // Add a new empty adjacency list
            }
        }

        // Add edges in both directions
        const size_t i = keyToIndex->at(keys[0]);
        const size_t j = keyToIndex->at(keys[1]);
        adjacencyList[i].push_back(j);
        adjacencyList[j].push_back(i);
    }

    return adjacencyList;
}

/**
 * Computes fundamental cycles in a factor graph using binary factors.
 * Returns cycles and tree edges with the original factor graph keys.
 */
template <typename FactorGraph>
FundamentalCyclesResult computeFundamentalCycles(const FactorGraph& graph, Key rootKey) {
    std::map<Key, size_t> keyToIndex;
    std::vector<size_t> indexToKey;

    GraphList adjList = buildAdjacencyListFromFactors(graph, &keyToIndex, &indexToKey);
    auto result = computeFundamentalCycles(adjList, keyToIndex.at(rootKey));

    // Translate indices back to original keys
    FundamentalCyclesResult keyResult;

    // Convert cycles
    for (const auto& cycle : result.cycles) {
        FundamentalCycle keyCycle;
        for (size_t idx : cycle.cycle) {
            keyCycle.cycle.push_back(indexToKey[idx]);
        }
        keyResult.cycles.push_back(keyCycle);
    }

    // Convert tree edges
    for (const auto& edge : result.tree) {
        keyResult.tree.emplace(
            indexToKey[edge.first],
            indexToKey[edge.second]
        );
    }

    return keyResult;
}

/* ************************************************************************* */
TEST(FundamentalCycles, AdjacencyList) {
    // Test using adjacency list
    GraphList graph = {
            {1, 2},    // neighbors of vertex 0
            {0, 2, 3}, // neighbors of vertex 1
            {0, 1, 3}, // neighbors of vertex 2
            {1, 2, 4}, // neighbors of vertex 3
            {3}        // neighbors of vertex 4
    };

    auto result = computeFundamentalCycles(graph, 0);

    // Verify the number of fundamental cycles
    EXPECT(result.cycles.size() == 2);

    // Verify the tree edges
    EXPECT(result.tree.size() == 4);
    std::set<std::pair<size_t, size_t>> expectedTree = {
        {0, 1},
        {0, 2},
        {1, 3},
        {3, 4}
    };

    EXPECT(result.tree == expectedTree);
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
    size_t numNodes = 5;

    auto result = computeFundamentalCycles(edges, numNodes, 0);

    // Verify the number of fundamental cycles
    EXPECT(result.cycles.size() == 2);
    for (const auto& cycle : result.cycles) {
        size_t i = cycle.cycle.front();
        size_t j = cycle.cycle.back();
        EXPECT(result.tree.find({ i, j }) == result.tree.end());
    }

    // Verify the tree edges
    EXPECT(result.tree.size() == 4);
    std::set<std::pair<size_t, size_t>> expectedTree = {
        {0, 1},
        {0, 2},
        {1, 3},
        {3, 4}
    };

    EXPECT(result.tree == expectedTree);
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

    const auto result = computeFundamentalCycles(g, X(1));
    // Verify the number of fundamental cycles
    EXPECT(result.cycles.size() == 3);

    // Verify the tree edges - note we test the actual symbolic keys now
    EXPECT(result.tree.size() == 3);
    std::set<std::pair<size_t, size_t>> expectedTree = {
        {X(1), X(2)},
        {X(1), X(3)},
        {X(1), X(4)}
    };
    EXPECT(result.tree == expectedTree);
}


/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
