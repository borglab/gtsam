/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file LeveledSpanningTree
 * @brief Fast spanning tree and fundamental cycles.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/types.h>

#include <vector>
#include <optional>
#include <map>

namespace gtsam {

    /**
     * @brief A breadth-first spanning tree that maintains level information.
     *
     * This data structure supports efficient computation of fundamental cycles by:
     * 1. Constructing a BFS tree from a root node
     * 2. Recording both parent and level information for each node
     * 3. Using level information to quickly compute paths to the least common ancestor
     *
     * This follows Paton's 1969 algorithm for finding fundamental cycles in undirected graphs,
     * in "An algorithm for finding a fundamental set of cycles for an undirected linear graph".
     * However, the level information allows O(V) computation of fundamental cycles where V is the
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
        LeveledSpanningTree(const Graph& graph, size_t root);

        /**
         * Constructs a leveled spanning tree using breadth-first search, templated version.
         * @param graph The edge list representation of the graph
         * @param root The root vertex for the spanning tree
         * Time complexity: O(V+E) where V is the number of vertices and E is the number of edges
         */
        template <typename EdgeType>
        static LeveledSpanningTree FromEdges(const std::vector<EdgeType>& edges, size_t root) {
            // Calculate the number of vertices.
            size_t maxVertex = 0;
            for (const auto& edge : edges) {
                maxVertex = std::max({ maxVertex, edge.i, edge.j });
            }
            const size_t n = maxVertex + 1;

            // Assert that we have at least n-1 vertices
            if (edges.size() < n - 1) {
                throw std::invalid_argument("Invalid edge list: need at least n-1 edges to form a tree");
            }

            // Now create the adjacency list representation
            Graph graph(n);
            for (const auto& edge : edges) {
                // Assuming an undirected graph.
                graph[edge.i].push_back(edge.j);
                graph[edge.j].push_back(edge.i);
            }
            return LeveledSpanningTree(graph, root);
        }

        /**
         * Constructs a leveled spanning tree using breadth-first search, factor graph version.
         * @param factorGraph any factor graph
         * @param root The root vertex for the spanning tree
         * @param keyToIndex Output parameter: maps keys to vertex indices
         * @param indexToKey Output parameter: maps vertex indices to keys
         * Time complexity: O(V+E) where V is the number of vertices and E is the number of edges
         */
        template <typename FactorGraph>
        static LeveledSpanningTree FromFactorGraph(const FactorGraph& factorGraph, Key root,
            std::map<Key, size_t>* keyToIndex,
            std::vector<size_t>* indexToKey) {
            keyToIndex->clear();
            indexToKey->clear();
            Graph graph;

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

            return LeveledSpanningTree(graph, keyToIndex->at(root));
        }

        /// Get the parent of a vertex in the spanning tree.
        size_t parent(size_t vertex) const;

        /// Checks if an edge is part of the spanning tree. Time complexity: O(1)
        bool contains(size_t u, size_t v) const;

        /// Get the level (distance from root) of a vertex.
        size_t level(size_t vertex) const;

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
        Cycle fundamentalCycle(size_t u, size_t v) const;

        // Cycles: Holds all computed fundamental cycles
        using Cycles = std::vector<Cycle>;

        /**
         * Compute all fundamental cycles given an adjacency list.
         * @return A list of fundamental cycles, each represented as a sequence of vertices
         * Running time: O(E·V) in the worst case.
         */
        Cycles allFundamentalCycles(const Graph& graph) const;

        /**
         * Compute all fundamental cycles given an edge list.
         * @return A list of fundamental cycles, each represented as a sequence of vertices
         * Overall running time: O(E·V) in the worst case.
         */
        template <typename EdgeType>
        Cycles allFundamentalCycles(const std::vector<EdgeType>& edges) const {
            Cycles result;

            // Compute fundamental cycles for each non-tree edge in O(E) iterations (each worst-case O(V))
            for (const auto& edge : edges) {
                if (!contains(edge.i, edge.j)) {
                    result.push_back(fundamentalCycle(edge.i, edge.j));
                }
            }

            return result;
        }

        /**
         * Computes fundamental cycles in a factor graph using binary factors.
         * @param factorGraph The factor graph to analyze
         * @param keyToIndex maps keys to vertex indices
         * @param indexToKey maps vertex indices to keys
         * @return A list of fundamental cycles, each represented as a sequence of keys
         * Overall running time: O(E·V) in the worst case.
         */
        template <typename FactorGraph>
        Cycles allFundamentalCycles(const FactorGraph& factorGraph,
            const std::map<Key, size_t>& keyToIndex, const std::vector<size_t>& indexToKey) const {
            Cycles result;

            // Process all binary factors directly
            for (const auto& factor : factorGraph) {
                if (!factor || factor->keys().size() != 2) continue;

                const auto& keys = factor->keys();
                size_t i = keyToIndex.at(keys[0]);
                size_t j = keyToIndex.at(keys[1]);

                if (!contains(i, j)) {
                    // Get the fundamental cycle in terms of indices
                    auto cycle = fundamentalCycle(i, j);

                    // Convert to keys
                    Cycle keyCycle;
                    for (size_t idx : cycle) keyCycle.push_back(indexToKey[idx]);

                    result.push_back(keyCycle);
                }
            }

            return result;
        }
    };
}