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
#include <gtsam/geometry/Rot3.h>

namespace gtsam {

typedef std::map<Key, std::vector<size_t> > KeyVectorMap;
typedef std::map<Key, Rot3 > KeyRotMap;

namespace InitializePose3 {

GTSAM_EXPORT GaussianFactorGraph buildLinearOrientationGraph(const NonlinearFactorGraph& g);

GTSAM_EXPORT Values normalizeRelaxedRotations(const VectorValues& relaxedRot3);

GTSAM_EXPORT Values computeOrientationsChordal(const NonlinearFactorGraph& pose3Graph);

GTSAM_EXPORT Values computeOrientationsGradient(const NonlinearFactorGraph& pose3Graph,
    const Values& givenGuess, size_t maxIter = 10000, const bool setRefFrame = true);

GTSAM_EXPORT void createSymbolicGraph(KeyVectorMap& adjEdgesMap, KeyRotMap& factorId2RotMap,
    const NonlinearFactorGraph& pose3Graph);

GTSAM_EXPORT Vector3 gradientTron(const Rot3& R1, const Rot3& R2, const double a, const double b);

GTSAM_EXPORT NonlinearFactorGraph buildPose3graph(const NonlinearFactorGraph& graph);

GTSAM_EXPORT Values computePoses(NonlinearFactorGraph& pose3graph,  Values& initialRot);

GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph);

GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph, const Values& givenGuess, bool useGradient = false);

} // end of namespace lago
} // end of namespace gtsam
