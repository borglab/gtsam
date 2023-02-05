/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  lago.h
 *  @brief Initialize Pose2 in a factor graph using LAGO
 *  (Linear Approximation for Graph Optimization). see papers:
 *
 *  L. Carlone, R. Aragues, J. Castellanos, and B. Bona, A fast and accurate
 *  approximation for planar pose graph optimization, IJRR, 2014.
 *
 *  L. Carlone, R. Aragues, J.A. Castellanos, and B. Bona, A linear approximation
 *  for graph-based simultaneous localization and mapping, RSS, 2011.
 *
 *  @param graph: nonlinear factor graph (can include arbitrary factors but we assume
 *  that there is a subgraph involving Pose2 and betweenFactors). Also in the current
 *  version we assume that there is an odometric spanning path (x0->x1, x1->x2, etc)
 *  and a prior on x0. This assumption can be relaxed by using the extra argument
 *  useOdometricPath = false, although this part of code is not stable yet.
 *  @return Values: initial guess from LAGO (only pose2 are initialized)
 *
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   May 14, 2014
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/graph.h>

namespace gtsam {
namespace lago {

typedef std::map<Key, double> key2doubleMap;

/**
 * Compute the cumulative orientations (without wrapping)
 * for all nodes wrt the root (root has zero orientation).
 */
GTSAM_EXPORT key2doubleMap computeThetasToRoot(
    const key2doubleMap& deltaThetaMap, const PredecessorMap<Key>& tree);

/**
 * Given a factor graph "g", and a spanning tree "tree", select the nodes belonging
 * to the tree and to g, and stores the factor slots corresponding to edges in the
 * tree and to chordsIds wrt this tree.
 * Also it computes deltaThetaMap which is a fast way to encode relative
 * orientations along the tree: for a node key2, s.t. tree[key2]=key1,
 * the value deltaThetaMap[key2] is relative orientation theta[key2]-theta[key1]
 */
GTSAM_EXPORT void getSymbolicGraph(
/*OUTPUTS*/std::vector<size_t>& spanningTreeIds, std::vector<size_t>& chordsIds,
    key2doubleMap& deltaThetaMap,
    /*INPUTS*/const PredecessorMap<Key>& tree, const NonlinearFactorGraph& g);

/** Linear factor graph with regularized orientation measurements */
GTSAM_EXPORT GaussianFactorGraph buildLinearOrientationGraph(
    const std::vector<size_t>& spanningTreeIds,
    const std::vector<size_t>& chordsIds, const NonlinearFactorGraph& g,
    const key2doubleMap& orientationsToRoot, const PredecessorMap<Key>& tree);

/** Given a "pose2" factor graph, find it's minimum spanning tree.
 * Note: all 'Pose2' Between factors are given equal weightage.
 * Note: assumes all the edges (factors) are Between factors
 */
GTSAM_EXPORT PredecessorMap<Key> findMinimumSpanningTree(
    const NonlinearFactorGraph& pose2Graph);

/** LAGO: Return the orientations of the Pose2 in a generic factor graph */
GTSAM_EXPORT VectorValues initializeOrientations(
    const NonlinearFactorGraph& graph, bool useOdometricPath = true);

/** Return the values for the Pose2 in a generic factor graph */
GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph,
    bool useOdometricPath = true);

/** Only correct the orientation part in initialGuess */
GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph,
    const Values& initialGuess);

} // end of namespace lago
} // end of namespace gtsam
