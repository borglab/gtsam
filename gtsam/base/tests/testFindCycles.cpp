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

#include <CppUnitLite/TestHarness.h>
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <utility>


  // GraphList: adjacency list representation (vector of vectors of int)
using GraphList = std::vector<std::vector<int>>;

// BFSTreeResult: Holds BFS parent and level arrays. (O(1))
struct BFSTreeResult {
    std::vector<int> parent; // parent[v] = parent of vertex v (-1 if none)
    std::vector<int> level;  // level[v] = distance from root
};

/** bfsTree: Performs a breadth-first search on the given adjacency list starting at root. O(V+E) */
BFSTreeResult bfsTree(const GraphList& graph, int root) {
    int n = graph.size();
    std::vector<int> parent(n, -1), level(n, -1);
    std::queue<int> queue;
    queue.push(root);
    level[root] = 0;

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();
        for (int neighbor : graph[current]) {
            if (level[neighbor] == -1) {
                level[neighbor] = level[current] + 1;
                parent[neighbor] = current;
                queue.push(neighbor);
            }
        }
    }
    return { parent, level };
}

/** buildAdjacencyList: Converts an edge list to an adjacency list. O(E) */
template <typename EdgeType>
GraphList buildAdjacencyList(const std::vector<EdgeType>& edges, int numNodes) {
    GraphList graph(numNodes);
    for (const auto& edge : edges) {
        // Assuming an undirected graph.
        graph[edge.i].push_back(static_cast<int>(edge.j));
        graph[edge.j].push_back(static_cast<int>(edge.i));
    }
    return graph;
}

// FundamentalCycle: Wraps a cycle (sequence of vertex indices). (O(1))
struct FundamentalCycle {
    std::vector<int> cycle;
};

/**
 * computeFundamentalCycle: Computes the fundamental cycle corresponding to a non-tree edge (u,v).
 * Worst-case O(V) if the tree depth is O(V).
 */
FundamentalCycle computeFundamentalCycle(int u, int v, const std::vector<int>& parent, const std::vector<int>& level) {
    std::vector<int> pathU, pathV;
    // Climb up from u until levels are equal.
    while (level[u] > level[v]) {
        pathU.push_back(u);
        u = parent[u];
    }
    // Climb up from v until levels are equal.
    while (level[v] > level[u]) {
        pathV.push_back(v);
        v = parent[v];
    }
    // Climb until the lowest common ancestor is reached.
    while (u != v) {
        pathU.push_back(u);
        pathV.push_back(v);
        u = parent[u];
        v = parent[v];
    }
    pathU.push_back(u); // Add the LCA.
    // Append the reverse of pathV to complete the cycle.
    pathU.insert(pathU.end(), pathV.rbegin(), pathV.rend());
    return { pathU };
}

// FundamentalCyclesResult: Holds all computed fundamental cycles and the tree edges. (O(1))
struct FundamentalCyclesResult {
    std::vector<FundamentalCycle> cycles;
    std::vector<std::pair<int, int>> treeEdges;
};

// Custom hash function for std::pair<int, int>
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ (std::hash<T2>()(pair.second) << 1);
    }
};

/**
 * computeFundamentalCycles:
 * Given an adjacency list, computes all fundamental cycles using a BFS tree.
 * Running time: O(V+E) for the BFS plus up to O(E*V) in the worst-case for cycle computation.
 */
FundamentalCyclesResult computeFundamentalCycles(const GraphList& graph, int root) {
    int n = graph.size();
    auto bfsResult = bfsTree(graph, root);
    const auto& parent = bfsResult.parent;
    const auto& level = bfsResult.level;

    // Collect tree edges in O(V)
    std::unordered_set<std::pair<int, int>, PairHash> treeEdgesSet;
    std::vector<std::pair<int, int>> treeEdges;
    for (int v = 0; v < n; ++v) {
        if (parent[v] != -1) {
            int a = std::min(v, parent[v]);
            int b = std::max(v, parent[v]);
            treeEdgesSet.insert({ a, b });
            treeEdges.push_back({ a, b });
        }
    }

    // Compute fundamental cycles for each non-tree edge in O(E) iterations (each worst-case O(V))
    std::vector<FundamentalCycle> cycles;
    for (int u = 0; u < n; ++u) {
        for (int v : graph[u]) {
            if (u < v && treeEdgesSet.find({ u, v }) == treeEdgesSet.end()) {
                cycles.push_back(computeFundamentalCycle(u, v, parent, level));
            }
        }
    }

    return { cycles, treeEdges };
}

/**
 * computeFundamentalCycles:
 * Given an edge list, converts it to an adjacency list (O(E)) and then computes fundamental cycles.
 * Overall running time: O(E + (Adjacency list version)).
 */
template <typename EdgeType>
FundamentalCyclesResult computeFundamentalCycles(const std::vector<EdgeType>& edges, int numNodes, int root) {
    GraphList graph = buildAdjacencyList(edges, numNodes); // O(E)
    return computeFundamentalCycles(graph, root);   // See function comment above.
}
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
    EXPECT(result.treeEdges.size() == 4);
    EXPECT(result.treeEdges[0] == std::make_pair(0, 1));
    EXPECT(result.treeEdges[1] == std::make_pair(0, 2));
    EXPECT(result.treeEdges[2] == std::make_pair(1, 3));
    EXPECT(result.treeEdges[3] == std::make_pair(3, 4));
}

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
    int numNodes = 5;

    auto result = computeFundamentalCycles(edges, numNodes, 0);

    // Verify the number of fundamental cycles
    EXPECT(result.cycles.size() == 2);

    // Verify the tree edges
    EXPECT(result.treeEdges.size() == 4);
    EXPECT(result.treeEdges[0] == std::make_pair(0, 1));
    EXPECT(result.treeEdges[1] == std::make_pair(0, 2));
    EXPECT(result.treeEdges[2] == std::make_pair(1, 3));
    EXPECT(result.treeEdges[3] == std::make_pair(3, 4));
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
