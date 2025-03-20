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


/**
 * LeveledSpanningTree: A breadth-first spanning tree that maintains level information.
 * This data structure supports efficient computation of fundamental cycles by:
 * 1. Constructing a BFS tree from a root node
 * 2. Recording both parent and level information for each node
 * 3. Using level information to quickly compute paths to the least common ancestor
 *
 * The level information allows O(V) computation of fundamental cycles where V is the
 * number of vertices in the graph. Without level information, finding the lowest common
 * ancestor would require O(V²) time in the worst case.
 */
class LeveledSpanningTree {
public:
    // Type aliases for graph structures
    using Graph = std::vector<std::vector<size_t>>;
    using Cycle = std::vector<size_t>; // Sequence of vertex indices forming a cycle
private:
    struct Node {
        size_t parent;  ///< Parent vertex in the spanning tree
        size_t level;   ///< Distance from the root
    };

    /// nodes[v] = {parent, level} for vertex v, or nullopt if unreachable
    std::vector<std::optional<Node>> nodes;

public:
    /**
     * Constructs a leveled spanning tree using breadth-first search.
     * @param graph The adjacency list representation of the graph
     * @param root The root vertex for the spanning tree
     * Time complexity: O(V+E) where V is the number of vertices and E is the number of edges
     */
    LeveledSpanningTree(const Graph& graph, size_t root) :nodes(graph.size(), std::nullopt) {
        // Root has no parent, so we use itself as parent
        nodes[root] = { root, 0 };

        std::queue<size_t> queue;
        queue.push(root);

        while (!queue.empty()) {
            size_t current = queue.front();
            queue.pop();

            size_t currentLevel = level(current);
            for (size_t neighbor : graph[current]) {
                if (!nodes[neighbor]) {
                    nodes[neighbor] = { current, currentLevel + 1 };
                    queue.push(neighbor);
                }
            }
        }
    }

    /// Get the level (distance from root) of a vertex.
    size_t level(size_t vertex) const {
        return nodes[vertex]->level;
    }

    /// Get the parent of a vertex in the spanning tree.
    size_t parent(size_t vertex) const {
        return nodes[vertex]->parent;
    }

    /**
     * Computes the fundamental cycle corresponding to a non-tree edge (u,v).
     * A fundamental cycle is formed by a non-tree edge plus the unique path
     * between its endpoints in the spanning tree.
     *
     * @param u First endpoint of the non-tree edge
     * @param v Second endpoint of the non-tree edge
     * @return The vertices in the fundamental cycle, starting with u, and ending with v
     * Time complexity: O(V) in the worst case (when the tree has depth O(V))
     */
    Cycle computeFundamentalCycle(size_t u, size_t v) const {
        std::vector<size_t> cycle, path;

        // Climb up from u until levels are equal
        while (level(u) > level(v)) {
            cycle.push_back(u);
            u = parent(u);
        }

        // Climb up from v until levels are equal
        while (level(v) > level(u)) {
            path.push_back(v);
            v = parent(v);
        }

        // Climb until the lowest common ancestor is reached
        while (u != v) {
            cycle.push_back(u);
            path.push_back(v);
            u = parent(u);
            v = parent(v);
        }

        cycle.push_back(u);  // Add the lowest common ancestor

        // Append the reverse of path to complete the cycle
        cycle.insert(cycle.end(), path.rbegin(), path.rend());

        return cycle;
    }
};

// FundamentalCyclesResult: Holds all computed fundamental cycles and the tree edges
struct FundamentalCyclesResult {
    std::vector<LeveledSpanningTree::Cycle> cycles;
    std::set<std::pair<size_t, size_t>> tree; // Tree edges as a set
};

/**
 * computeFundamentalCycles:
 * Given an adjacency list, computes all fundamental cycles using a BFS tree.
 * Running time: O(V+E) for the BFS plus up to O(E*V) in the worst-case for cycle computation.
 */
FundamentalCyclesResult computeFundamentalCycles(const LeveledSpanningTree::Graph& graph, size_t root) {
    size_t n = graph.size();
    const LeveledSpanningTree lst(graph, root);

    // Initialize result
    FundamentalCyclesResult result;

    // Collect tree edges in O(V)
    for (size_t v = 0; v < n; ++v) {
        auto parent = lst.parent(v);
        if (parent != v) {
            size_t a = std::min(v, parent);
            size_t b = std::max(v, parent);
            result.tree.emplace(a, b);
        }
    }

    // Compute fundamental cycles for each non-tree edge in O(E) iterations (each worst-case O(V))
    for (size_t u = 0; u < n; ++u) {
        for (size_t v : graph[u]) {
            if (u < v && result.tree.find({ u, v }) == result.tree.end()) {
                result.cycles.push_back(lst.computeFundamentalCycle(u, v));
            }
        }
    }

    return result;
}

/** buildAdjacencyList: Converts an edge list to an adjacency list. O(E) */
template <typename EdgeType>
LeveledSpanningTree::Graph buildAdjacencyList(const std::vector<EdgeType>& edges, size_t numNodes) {
    LeveledSpanningTree::Graph graph(numNodes);
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
    LeveledSpanningTree::Graph graph = buildAdjacencyList(edges, numNodes); // O(E)
    return computeFundamentalCycles(graph, root);   // See function comment above.
}
/**
 * buildAdjacencyListFromFactors: Builds an adjacency list from binary factors in a graph. O(F)
 */
template <typename FactorGraph>
LeveledSpanningTree::Graph buildAdjacencyListFromFactors(const FactorGraph& factorGraph,
    std::map<Key, size_t>* keyToIndex,
    std::vector<size_t>* indexToKey) {
    keyToIndex->clear();
    indexToKey->clear();
    LeveledSpanningTree::Graph graph;

    // Process all binary factors in a single pass
    for (const auto& factor : factorGraph) {
        if (!factor || factor->keys().size() != 2) continue;

        const auto& keys = factor->keys();

        // Get or create indices for both keys
        for (const auto& key : keys) {
            if (keyToIndex->find(key) == keyToIndex->end()) {
                const size_t n = keyToIndex->size();
                keyToIndex->emplace(key, n);
                indexToKey->push_back(key);
                graph.push_back({}); // Add a new empty adjacency list
            }
        }

        // Add edges in both directions
        const size_t i = keyToIndex->at(keys[0]);
        const size_t j = keyToIndex->at(keys[1]);
        graph[i].push_back(j);
        graph[j].push_back(i);
    }

    return graph;
}

/**
 * Computes fundamental cycles in a factor graph using binary factors.
 * Returns cycles and tree edges with the original factor graph keys.
 */
template <typename FactorGraph>
FundamentalCyclesResult computeFundamentalCycles(const FactorGraph& graph, Key root) {
    std::map<Key, size_t> keyToIndex;
    std::vector<size_t> indexToKey;

    LeveledSpanningTree::Graph adjList = buildAdjacencyListFromFactors(graph, &keyToIndex, &indexToKey);
    auto result = computeFundamentalCycles(adjList, keyToIndex.at(root));

    // Translate indices back to original keys
    FundamentalCyclesResult keyResult;

    // Convert cycles
    for (const auto& cycle : result.cycles) {
        LeveledSpanningTree::Cycle keyCycle;
        for (size_t idx : cycle) {
            keyCycle.push_back(indexToKey[idx]);
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
    LeveledSpanningTree::Graph graph = {
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
        size_t i = cycle.front();
        size_t j = cycle.back();
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
