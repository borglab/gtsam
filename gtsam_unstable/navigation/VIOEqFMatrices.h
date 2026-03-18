/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VIOEqFMatrices.h
 * @brief   EqF coordinate suites for VIO foundations
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>
#include <gtsam_unstable/navigation/EqVIOState.h>
#include <gtsam_unstable/dllexport.h>

#include <functional>
#include <memory>

namespace gtsam {
namespace eqvio {

enum class CoordinateChoice { Euclidean, InvDepth };

struct GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite {
  /// Coordinate chart about xi0 and inverse chart.
  std::function<Vector(const VIOState&, const VIOState&)> stateChart;
  std::function<VIOState(const Vector&, const VIOState&)> stateChartInv;

  /// Continuous-time matrices.
  std::function<Matrix(const VIOGroup&, const VIOState&, const IMUInput&)>
      stateMatrixA;
  std::function<Matrix(const VIOGroup&, const VIOState&)> inputMatrixB;

  /// Output matrix block C_i^*.
  std::function<Matrix23(const Point3&, const SOT3&,
                         const std::shared_ptr<const VIOCameraModel>&,
                         const Point2&)>
      outputMatrixCiStar;

  /// Lift maps from innovation coordinates.
  std::function<Vector(const Vector&, const VIOState&)> liftInnovation;
  std::function<VIOGroup(const Vector&, const VIOState&)> liftInnovationDiscrete;

  Matrix outputMatrixC(const VIOState& xi0, const VIOGroup& X,
                       const VisionMeasurement& y,
                       const std::shared_ptr<const VIOCameraModel>& camera,
                       bool useEquivariance = true) const;

  Matrix stateMatrixADiscrete(const VIOGroup& X, const VIOState& xi0,
                              const IMUInput& imuVel, double dt) const;

  Matrix23 outputMatrixCi(const Point3& q0, const SOT3& QHat,
                          const std::shared_ptr<const VIOCameraModel>& camera)
      const;
};

extern const GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite EqFCoordinateSuite_euclid;
extern const GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite
    EqFCoordinateSuite_invdepth;

const GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite* getCoordinates(
    CoordinateChoice coordinateChoice);

}  // namespace eqvio
}  // namespace gtsam
