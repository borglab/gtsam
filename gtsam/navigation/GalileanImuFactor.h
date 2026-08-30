/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanImuFactor.h
 * @brief   Left-invariant Galilean IMU preintegration factor aliases
 */

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GalileanPreintegration.h>
#include <gtsam/navigation/ImuFactor.h>

namespace gtsam {

/// Galilean preintegration with generic PIM covariance propagation.
using PreintegratedImuMeasurementsG =
    PreintegratedImuMeasurementsT<GalileanPreintegration>;

/// Five-way IMU factor using Galilean preintegration.
using GalileanImuFactor = ImuFactorT<PreintegratedImuMeasurementsG>;

/// Three-way NavState IMU factor using Galilean preintegration.
using GalileanImuFactor2 = ImuFactor2T<PreintegratedImuMeasurementsG>;

/// Galilean preintegration with combined IMU and bias covariance propagation.
using PreintegratedCombinedMeasurementsG =
    PreintegratedCombinedMeasurementsT<GalileanPreintegration>;

/// Six-way Combined IMU factor using Galilean preintegration.
using GalileanCombinedImuFactor =
    CombinedImuFactorT<PreintegratedCombinedMeasurementsG>;

}  // namespace gtsam
