/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmTypes.h
 * @brief   Trivially copyable measurement and noise types for CUDA SFM
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

namespace gtsam::cuda {

/**
 * One image measurement, flattened for the device.
 *
 * Keys are replaced by dense slot indices into the packed camera and point
 * arrays, since a kernel cannot look up a gtsam::Key. Every field is a
 * fixed-width scalar so a whole graph's observations upload as one copy.
 */
struct SfmObservation {
  /// Index into the packed camera array.
  int cameraSlot;
  /// Index into the packed point array.
  int pointSlot;
  /// Measured pixel column.
  double measuredU;
  /// Measured pixel row.
  double measuredV;
};

/**
 * Upper-triangular square root information matrix R for one 2D measurement,
 * which whitens its residual as R·r and its Jacobian as R·J.
 *
 * This is what a Gaussian noise model contributes on the host, reduced to the
 * three structurally nonzero entries of a 2x2 upper triangle.
 */
struct SfmSqrtInfo2 {
  /// Row 0, column 0.
  double r00;
  /// Row 0, column 1.
  double r01;
  /// Row 1, column 1.
  double r11;
};

/// Robust loss applied to whitened errors, or None for plain least squares.
enum class SfmRobustModelKind {
  None,
  Huber,
  Tukey,
};

/// Whether a robust weight scales the 2D residual as a block or per component.
enum class SfmRobustReweightScheme {
  Scalar,
  Block,
};

/// Robust loss selection and tuning constant, passed by value into kernels.
struct SfmRobustModel {
  /// Which loss to apply.
  SfmRobustModelKind kind;
  /// How the resulting weight is applied to the 2D residual block.
  SfmRobustReweightScheme reweightScheme;
  /// The loss's threshold constant, unused when kind is None.
  double parameter;
};

}  // namespace gtsam::cuda
