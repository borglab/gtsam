/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3.h
 * @brief  Common code for all Calibration models.
 * @author Varun Agrawal
 */

/**
 * @addtogroup geometry
 */

#pragma once

#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * Function which makes use of the Implicit Function Theorem to compute the
 * Jacobians of `calibrate` using `uncalibrate`.
 * This is useful when there are iterative operations in the `calibrate`
 * function which make computing jacobians difficult.
 * 
 * Given f(pi, pn) = uncalibrate(pn) - pi, and g(pi) = calibrate, we can
 * easily compute the Jacobians:
 * df/pi = -I (pn and pi are independent args)
 * Dp = -inv(H_uncal_pn) * df/pi = -inv(H_uncal_pn) * (-I) = inv(H_uncal_pn)
 * Dcal = -inv(H_uncal_pn) * df/K = -inv(H_uncal_pn) * H_uncal_K
 *
 * @tparam Cal Calibration model.
 * @tparam Dim The number of parameters in the calibration model.
 * @param p Calibrated point.
 * @param Dcal optional 2*p Jacobian wrpt `p` Cal3DS2 parameters.
 * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates.
 */
template <typename Cal, size_t Dim>
void calibrateJacobians(const Cal& calibration, const Point2& pn,
                        OptionalJacobian<2, Dim> Dcal = boost::none,
                        OptionalJacobian<2, 2> Dp = boost::none) {
  if (Dcal || Dp) {
    Eigen::Matrix<double, 2, Dim> H_uncal_K;
    Matrix22 H_uncal_pn, H_uncal_pn_inv;

    // Compute uncalibrate Jacobians
    calibration.uncalibrate(pn, Dcal ? &H_uncal_K : nullptr, H_uncal_pn);

    H_uncal_pn_inv = H_uncal_pn.inverse();

    if (Dp) *Dp = H_uncal_pn_inv;
    if (Dcal) *Dcal = -H_uncal_pn_inv * H_uncal_K;
  }
}

//TODO(Varun) Make common base class for all calibration models.

}  // \ namespace gtsam
