/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  InitializePose.h
 *  @author Frank Dellaert
 *  @date   August, 2020
 *  @brief common code between lago.* (2D) and InitializePose3.* (3D)
 */

#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace gtsam {
namespace initialize {

static constexpr Key kAnchorKey = 99999999;

/**
 * Select the subgraph of betweenFactors and transforms priors into between
 * wrt a fictitious node
 */
template <class Pose>
static NonlinearFactorGraph buildPoseGraph(const NonlinearFactorGraph& graph) {
  NonlinearFactorGraph poseGraph;

  for (const auto& factor : graph) {
    // recast to a between on Pose
    if (auto between =
            std::dynamic_pointer_cast<BetweenFactor<Pose> >(factor))
      poseGraph.add(between);

    // recast PriorFactor<Pose> to BetweenFactor<Pose>
    if (auto prior = std::dynamic_pointer_cast<PriorFactor<Pose> >(factor))
      poseGraph.emplace_shared<BetweenFactor<Pose> >(
          kAnchorKey, prior->keys()[0], prior->prior(), prior->noiseModel());
  }
  return poseGraph;
}

/**
 * Use Gauss-Newton optimizer to optimize for poses given rotation estimates
 */
template <class Pose>
static Values computePoses(const Values& initialRot,
                           NonlinearFactorGraph* posegraph,
                           bool singleIter = true) {
  const auto origin = Pose().translation();

  // Upgrade rotations to full poses
  Values initialPose;
  for (const auto key_value : initialRot) {
    Key key = key_value.key;
    const auto& rot = initialRot.at<typename Pose::Rotation>(key);
    Pose initializedPose = Pose(rot, origin);
    initialPose.insert(key, initializedPose);
  }

  // add prior on dummy node
  auto priorModel = noiseModel::Unit::Create(Pose::dimension);
  initialPose.insert(kAnchorKey, Pose());
  posegraph->emplace_shared<PriorFactor<Pose> >(kAnchorKey, Pose(), priorModel);

  // Create optimizer
  GaussNewtonParams params;
  if (singleIter) {
    params.maxIterations = 1;
  } else {
    params.setVerbosity("TERMINATION");
  }
  GaussNewtonOptimizer optimizer(*posegraph, initialPose, params);
  Values GNresult = optimizer.optimize();

  // put into Values structure
  Values estimate;
  for (const auto key_value : GNresult) {
    Key key = key_value.key;
    if (key != kAnchorKey) {
      const Pose& pose = GNresult.at<Pose>(key);
      estimate.insert(key, pose);
    }
  }
  return estimate;
}
}  // namespace initialize
}  // namespace gtsam
