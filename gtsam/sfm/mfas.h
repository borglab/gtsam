/*
  This file defines functions used to solve a Minimum feedback arc set (MFAS)
  problem. This code was forked and modified from Kyle Wilson's repository at
  https://github.com/wilsonkl/SfM_Init.
  Copyright (c) 2014, Kyle Wilson
  All rights reserved.

  Given a weighted directed graph, the objective in a Minimum feedback arc set
  problem is to obtain a graph that does not contain any cycles by removing
  edges such that the total weight of removed edges is minimum.
*/
#ifndef __MFAS_H__
#define __MFAS_H__

#include <gtsam/inference/Key.h>

#include <map>
#include <vector>

namespace gtsam {

using KeyPair = std::pair<Key, Key>;

namespace mfas {

/*
 * Given a vector of KeyPairs that constitutes edges in a graph and the weights corresponding to these edges, this
 * function changes all the weights to positive numbers by flipping the direction of the edges that have a
 * negative weight. The changes are made in place.
 * @param edges reference to vector of KeyPairs
 * @param weights weights corresponding to edges
 */
void flipNegEdges(std::vector<KeyPair> &edges, std::vector<double> &weights);

/*
 * Computes the MFAS ordering, ie an ordering of the nodes in the graph such that the source of any edge appears before its destination in the ordering. The weight of edges that are removed to obtain this ordering is minimized.
 * @param edges: edges in the graph
 * @param weights: weights corresponding to the edges (have to be positive)
 * @param nodes: nodes in the graph
 * @param ordered_positions: map from node to position in the ordering (0 indexed)
 */
void mfasRatio(const std::vector<KeyPair> &edges,
               const std::vector<double> &weights, const KeyVector &nodes,
               FastMap<Key, int> &ordered_positions);

/*
 * Returns the weights of edges that are not consistent with the input ordering.
 * @param edges in the graph
 * @param weights of the edges in the graph
 * @param ordered_positions: ordering (obtained from MFAS solution)
 * @param broken: reference to a map from edges to their "broken weights"
 */
void brokenWeights(const std::vector<KeyPair> &edges,
                   const std::vector<double> &weight,
                   const FastMap<Key, int> &ordered_positions,
                   FastMap<Key, double> &broken);

}  // namespace mfas
}  // namespace gtsam
#endif  // __MFAS_H__
