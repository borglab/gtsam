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

GTSAM_EXPORT Values computeOrientationsChordal(const NonlinearFactorGraph& pose3Graph);

GTSAM_EXPORT Values computeOrientationsGradient(const NonlinearFactorGraph& pose3Graph, const Values& givenGuess);

GTSAM_EXPORT NonlinearFactorGraph buildPose3graph(const NonlinearFactorGraph& graph);

GTSAM_EXPORT Values computePoses(NonlinearFactorGraph& pose3graph,  Values& initialRot);

GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph);

GTSAM_EXPORT Values initialize(const NonlinearFactorGraph& graph, const Values& givenGuess, bool useGradient = false);

} // end of namespace lago
} // end of namespace gtsam
