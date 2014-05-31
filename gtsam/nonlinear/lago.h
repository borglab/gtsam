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

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/graph.h>

namespace gtsam {
namespace lago {

typedef std::map<Key,double> key2doubleMap;
const Key keyAnchor = symbol('Z',9999999);
noiseModel::Diagonal::shared_ptr priorOrientationNoise = noiseModel::Diagonal::Variances((Vector(1) << 1e-8));
noiseModel::Diagonal::shared_ptr priorPose2Noise = noiseModel::Diagonal::Variances((Vector(3) << 1e-6, 1e-6, 1e-8));

/*  This function computes the cumulative orientation (without wrapping) wrt the root of a spanning tree (tree)
 *  for a node (nodeKey). The function starts at the nodes and moves towards the root
 *  summing up the (directed) rotation measurements. Relative measurements are encoded in "deltaThetaMap"
 *  The root is assumed to have orientation zero.
 */
GTSAM_EXPORT double computeThetaToRoot(const Key nodeKey, const PredecessorMap<Key>& tree,
    const key2doubleMap& deltaThetaMap, const key2doubleMap& thetaFromRootMap);

/*  This function computes the cumulative orientations (without wrapping)
 *  for all node wrt the root (root has zero orientation)
 */
GTSAM_EXPORT key2doubleMap computeThetasToRoot(const key2doubleMap& deltaThetaMap,
    const PredecessorMap<Key>& tree);

/*  Given a factor graph "g", and a spanning tree "tree", the function selects the nodes belonging to the tree and to g,
 *  and stores the factor slots corresponding to edges in the tree and to chordsIds wrt this tree
 *  Also it computes deltaThetaMap which is a fast way to encode relative orientations along the tree:
 *  for a node key2, s.t. tree[key2]=key1, the values deltaThetaMap[key2] is the relative orientation theta[key2]-theta[key1]
 */
GTSAM_EXPORT void getSymbolicGraph(
    /*OUTPUTS*/ std::vector<size_t>& spanningTreeIds, std::vector<size_t>& chordsIds, key2doubleMap& deltaThetaMap,
    /*INPUTS*/ const PredecessorMap<Key>& tree, const NonlinearFactorGraph& g);

/*  Retrieves the deltaTheta and the corresponding noise model from a BetweenFactor<Pose2> */
GTSAM_EXPORT void getDeltaThetaAndNoise(NonlinearFactor::shared_ptr factor,
    Vector& deltaTheta, noiseModel::Diagonal::shared_ptr& model_deltaTheta);

/*  Linear factor graph with regularized orientation measurements */
GTSAM_EXPORT GaussianFactorGraph buildLinearOrientationGraph(const std::vector<size_t>& spanningTreeIds, const std::vector<size_t>& chordsIds,
    const NonlinearFactorGraph& g, const key2doubleMap& orientationsToRoot, const PredecessorMap<Key>& tree);

/* Selects the subgraph of betweenFactors and transforms priors into between wrt a fictitious node */
GTSAM_EXPORT NonlinearFactorGraph buildPose2graph(const NonlinearFactorGraph& graph);

/* Returns the orientations of a graph including only BetweenFactors<Pose2> */
GTSAM_EXPORT VectorValues computeLagoOrientations(const NonlinearFactorGraph& pose2Graph, bool useOdometricPath = true);

/*  LAGO: Returns the orientations of the Pose2 in a generic factor graph */
GTSAM_EXPORT VectorValues initializeOrientationsLago(const NonlinearFactorGraph& graph, bool useOdometricPath = true);

/*  Returns the values for the Pose2 in a generic factor graph */
GTSAM_EXPORT Values initializeLago(const NonlinearFactorGraph& graph, bool useOdometricPath = true);

/*  Only corrects the orientation part in initialGuess */
GTSAM_EXPORT Values initializeLago(const NonlinearFactorGraph& graph, const Values& initialGuess);

} // end of namespace lago
} // end of namespace gtsam
