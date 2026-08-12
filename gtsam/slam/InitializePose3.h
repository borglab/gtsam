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

#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>
#include <vector>

namespace gtsam {

typedef std::map<Key, std::vector<size_t> > KeyVectorMap;
typedef std::map<Key, Rot3> KeyRotMap;

struct GTSAM_EXPORT InitializePose3 {
  static GaussianFactorGraph buildLinearOrientationGraph(
      const NonlinearFactorGraph& g);

  static Values normalizeRelaxedRotations(const VectorValues& relaxedRot3);

  /**
   * Return the orientations of a graph including only BetweenFactors<Pose3>
   */
  static Values computeOrientationsChordal(
      const NonlinearFactorGraph& pose3Graph);

  /**
   * Return the orientations of a graph including only BetweenFactors<Pose3>
   */
  static Values computeOrientationsGradient(
      const NonlinearFactorGraph& pose3Graph, const Values& givenGuess,
      size_t maxIter = 10000, const bool setRefFrame = true);

  static void createSymbolicGraph(const NonlinearFactorGraph& pose3Graph,
                                  KeyVectorMap* adjEdgesMap,
                                  KeyRotMap* factorId2RotMap);

  static Vector3 gradientTron(const Rot3& R1, const Rot3& R2, const double a,
                              const double b);

  /**
   * Select the subgraph of betweenFactors and transforms priors into between
   * wrt a fictitious node
   */
  static NonlinearFactorGraph buildPose3graph(
      const NonlinearFactorGraph& graph);

  /**
   * Use Gauss-Newton optimizer to optimize for poses given rotation estimates
   */
  static Values computePoses(const Values& initialRot,
                             NonlinearFactorGraph* poseGraph,
                             bool singleIter = true);

  /**
   * "extract" the Pose3 subgraph of the original graph, get orientations from
   * relative orientation measurements using chordal method.
   */
  static Values initializeOrientations(const NonlinearFactorGraph& graph);

  /**
   * "extract" the Pose3 subgraph of the original graph, get orientations from
   * relative orientation measurements (using either gradient or chordal
   * method), and finish up with 1 GN iteration on full poses.
   */
  static Values initialize(const NonlinearFactorGraph& graph,
                           const Values& givenGuess, bool useGradient = false);

  /// Calls initialize above using Chordal method.
  static Values initialize(const NonlinearFactorGraph& graph);
};
}  // end of namespace gtsam
