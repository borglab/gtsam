/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  InitializePose3.h
 *  @brief Initialize Pose3 in a factor graph
 *
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   August, 2014
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/graph.h>

namespace gtsam {
namespace InitializePose3 {

GTSAM_EXPORT GaussianFactorGraph buildLinearOrientationGraph(const NonlinearFactorGraph& g);

GTSAM_EXPORT Values normalizeRelaxedRotations(const VectorValues& relaxedRot3);

GTSAM_EXPORT Values computeOrientations(const NonlinearFactorGraph& pose3Graph);

GTSAM_EXPORT Values initializeOrientations(const NonlinearFactorGraph& graph);

GTSAM_EXPORT NonlinearFactorGraph buildPose3graph(const NonlinearFactorGraph& graph);

GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph, const Values& givenGuess);

///** Linear factor graph with regularized orientation measurements */
//GTSAM_EXPORT GaussianFactorGraph buildLinearOrientationGraph(
//    const std::vector<size_t>& spanningTreeIds,
//    const std::vector<size_t>& chordsIds, const NonlinearFactorGraph& g,
//    const key2doubleMap& orientationsToRoot, const PredecessorMap<Key>& tree);
//
///** LAGO: Return the orientations of the Pose2 in a generic factor graph */
//GTSAM_EXPORT VectorValues initializeOrientations(
//    const NonlinearFactorGraph& graph, bool useOdometricPath = true);
//
///** Return the values for the Pose2 in a generic factor graph */
//GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph,
//    bool useOdometricPath = true);
//
///** Only correct the orientation part in initialGuess */
//GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph,
//    const Values& initialGuess);

} // end of namespace lago
} // end of namespace gtsam
