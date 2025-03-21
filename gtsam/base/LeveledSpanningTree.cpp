/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file LeveledSpanningTree.cpp
 * @brief Implementation of the LeveledSpanningTree class.
 * @author Frank Dellaert
 */

#include <gtsam/base/LeveledSpanningTree.h>
#include <queue>

namespace gtsam {

    LeveledSpanningTree::LeveledSpanningTree(const Graph& graph, size_t root)
        : nodes(graph.size(), std::nullopt) {
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

    size_t LeveledSpanningTree::parent(size_t vertex) const {
        return nodes[vertex]->parent;
    }

    bool LeveledSpanningTree::contains(size_t u, size_t v) const {
        return (nodes[u] && nodes[v] &&
            (nodes[u]->parent == v || nodes[v]->parent == u));
    }

    size_t LeveledSpanningTree::level(size_t vertex) const {
        return nodes[vertex]->level;
    }

    LeveledSpanningTree::Cycle LeveledSpanningTree::fundamentalCycle(size_t u, size_t v) const {
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

    LeveledSpanningTree::Cycles LeveledSpanningTree::allFundamentalCycles(const Graph& graph) const {
        Cycles result;

        // Compute fundamental cycles for each non-tree edge in O(E) iterations (each worst-case O(V))
        for (size_t u = 0; u < graph.size(); ++u) {
            for (size_t v : graph[u]) {
                if (u < v && !contains(u, v)) {
                    result.push_back(fundamentalCycle(u, v));
                }
            }
        }

        return result;
    }

} // namespace gtsam