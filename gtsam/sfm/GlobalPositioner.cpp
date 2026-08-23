/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GlobalPositioner.cpp
 * @author Kathir Gounder
 * @date 2026
 * @brief GLOMAP-style joint estimation of camera and landmark positions.
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/sfm/GlobalPositioner.h>

#include <stdexcept>

using namespace gtsam;
using namespace std;

Values GlobalPositioner::initializeRandomly(
    const std::set<Key> &cameraKeys, const std::set<Key> &landmarkKeys,
    const CameraPointDirections &cameraPointDirections,
    const Values &initialValues) const {
  std::set<Key> allKeys(cameraKeys);
  allKeys.insert(landmarkKeys.begin(), landmarkKeys.end());
  return LocationRecovery::initializeRandomly(allKeys,
      cameraPointDirections.size(), true, initialValues);
}

Values GlobalPositioner::run(
    const CameraPointDirections &cameraPointDirections,
    const std::set<Key> &cameraKeys, const std::set<Key> &landmarkKeys,
    Key anchorCameraKey, const Values &initialValues) const {
  if (cameraKeys.find(anchorCameraKey) == cameraKeys.end()) {
    throw std::invalid_argument(
        "GlobalPositioner::run: anchorCameraKey must be in cameraKeys.");
  }

  NonlinearFactorGraph graph = buildGraph(cameraPointDirections, true);
  addAnchorPrior(anchorCameraKey, &graph);
  Values initial =
      initializeRandomly(cameraKeys, landmarkKeys, cameraPointDirections,
                          initialValues);

  LevenbergMarquardtOptimizer lm(graph, initial, lmParams_);
  return lm.optimize();
}
