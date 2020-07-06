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
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <vector>

namespace gtsam {

using KeyPair = std::pair<Key, Key>;

namespace mfas {

void flipNegEdges(std::vector<KeyPair> &edges, std::vector<double> &weights);

void mfasRatio(const std::vector<KeyPair> &edges,
               const std::vector<double> &weights, const KeyVector &nodes,
               FastMap<Key, int> &ordered_positions);

void brokenWeights(const std::vector<KeyPair> &edges,
                   const std::vector<double> &weight,
                   const FastMap<Key, int> &ordered_positions,
                   FastMap<Key, double> &broken);

}  // namespace mfas
}  // namespace gtsam
#endif  // __MFAS_H__
