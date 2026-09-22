/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SnavelyProjection.h
 * @brief Ceres-compatible projection functor for BAL tests and benchmarks.
 */

#pragma once

#include <ceres/rotation.h>

namespace gtsam::testing {

/// Projects a BAL point using Snavely's nine-parameter camera model.
struct SnavelyProjection {
  /** Projects `point` through `camera` into `predicted`. */
  template <typename T>
  bool operator()(const T* camera, const T* point, T* predicted) const {
    T cameraPoint[3];
    ceres::AngleAxisRotatePoint(camera, point, cameraPoint);

    cameraPoint[0] += camera[3];
    cameraPoint[1] += camera[4];
    cameraPoint[2] += camera[5];

    const T normalizedX = -cameraPoint[0] / cameraPoint[2];
    const T normalizedY = -cameraPoint[1] / cameraPoint[2];
    const T radiusSquared =
        normalizedX * normalizedX + normalizedY * normalizedY;
    const T distortion =
        T(1.0) + radiusSquared * (camera[7] + camera[8] * radiusSquared);

    predicted[0] = camera[6] * distortion * normalizedX;
    predicted[1] = camera[6] * distortion * normalizedY;
    return true;
  }
};

}  // namespace gtsam::testing
