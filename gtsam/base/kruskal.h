/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file kruskal.h
 * @date Dec 31, 2009
 * @author Frank Dellaert
 * @author Yong-Dian Jian
 * @author Ankur Roy Chowdhury
 */

#pragma once

#include <gtsam/base/FastMap.h>
#include <gtsam/inference/FactorGraph.h>

#include <vector>

namespace gtsam::utils {
/**
 * Compute the minimum spanning tree (MST) using Kruskal's algorithm
 * @param fg Factor graph
 * @param weights Weights of the edges/factors in the factor graph
 * @return Edge/factor indices spanning the MST
 * @note Only binary factors are considered while constructing the spanning tree 
 * @note The indices of 'weights' should match the indices of the edges in the factor graph 
 */
template <class FACTOR>
std::vector<size_t> kruskal(const FactorGraph<FACTOR> &fg,
                            const std::vector<double> &weights);
}  // namespace gtsam::utils

#include <gtsam/base/kruskal-inl.h>
