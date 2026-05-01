/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file GlobalPositioner.h
 * @author Kathir Gounder
 * @date 2026
 * @brief GLOMAP-style joint estimation of camera and landmark positions
 *        from camera-to-point direction measurements.
 */

#include <gtsam/sfm/LocationRecovery.h>

#include <set>

namespace gtsam {

// Opinionated composition of LocationRecovery for the bipartite
// camera+landmark estimation problem (GLOMAP-style global positioning).
// Enforces:
//   - Explicit separation of camera keys and landmark keys
//   - Validation that the anchor key is a camera
//   - BILINEAR (BATA) cost function exclusively
//   - No DSFMap handling (not applicable to camera-landmark edges)
//
// For custom graph configurations (mixed edges, custom gauge, etc.),
// use LocationRecovery directly.
//
// Reference: Pan, L. et al., "Global Structure-from-Motion Revisited,"
// ECCV 2024.
class GTSAM_EXPORT GlobalPositioner : public LocationRecovery {
 public:
  using CameraPointDirection = BinaryMeasurement<Unit3>;
  using CameraPointDirections = std::vector<CameraPointDirection>;

  /**
   * @brief Construct with LM parameters.
   */
  explicit GlobalPositioner(const LevenbergMarquardtParams &lmParams)
      : LocationRecovery(lmParams) {}

  /**
   * @brief Default constructor.
   */
  GlobalPositioner() = default;

  /**
   * @brief Initialize cameras, landmarks, and BATA scale variables.
   * Unions cameraKeys and landmarkKeys, derives numEdges from measurements.
   */
  Values initializeRandomly(
      const std::set<Key> &cameraKeys, const std::set<Key> &landmarkKeys,
      const CameraPointDirections &cameraPointDirections,
      const Values &initialValues = Values()) const;

  /**
   * @brief Build graph, fix gauge, initialize, and optimize.
   *
   * Enforces bipartite structure: anchorCameraKey must be in cameraKeys.
   * For custom configurations, use LocationRecovery directly.
   */
  Values run(const CameraPointDirections &cameraPointDirections,
             const std::set<Key> &cameraKeys,
             const std::set<Key> &landmarkKeys, Key anchorCameraKey,
             const Values &initialValues = Values()) const;
};

}  // namespace gtsam
