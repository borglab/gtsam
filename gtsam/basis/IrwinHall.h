/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IrwinHall.h
 * @brief Piecewise polynomial Irwin-Hall PDF and CDF kernels.
 * @author Brett Downing
 */

#pragma once

#include <gtsam/basis/PiecewisePolynomial.h>
#include <gtsam/dllexport.h>

namespace gtsam {
namespace kernels {

/**
 * Irwin-Hall kernels for uniform cardinal splines.
 *
 * The PDFs below follow OEIS A188816 and the CDFs follow A188668. The CDF
 * index is two less than the resulting polynomial spline degree: CDF0 gives
 * linear interpolation and CDF2 gives a cubic spline.
 */

/// Order-zero Irwin-Hall PDF.
extern GTSAM_EXPORT const PiecewisePolynomial<0, 1> IrwinHall0;
/// Order-one Irwin-Hall PDF.
extern GTSAM_EXPORT const PiecewisePolynomial<1, 2> IrwinHall1;
/// Order-two Irwin-Hall PDF.
extern GTSAM_EXPORT const PiecewisePolynomial<2, 3> IrwinHall2;
/// Order-three Irwin-Hall PDF.
extern GTSAM_EXPORT const PiecewisePolynomial<3, 4> IrwinHall3;
/// Order-four Irwin-Hall PDF.
extern GTSAM_EXPORT const PiecewisePolynomial<4, 5> IrwinHall4;
/// Order-five Irwin-Hall PDF.
extern GTSAM_EXPORT const PiecewisePolynomial<5, 6> IrwinHall5;
/// Order-six Irwin-Hall PDF.
extern GTSAM_EXPORT const PiecewisePolynomial<6, 7> IrwinHall6;

/// Linear cardinal-spline CDF kernel.
extern GTSAM_EXPORT const PiecewisePolynomial<1, 1> IrwinHallCDF0;
/// Quadratic cardinal-spline CDF kernel.
extern GTSAM_EXPORT const PiecewisePolynomial<2, 2> IrwinHallCDF1;
/// Cubic cardinal-spline CDF kernel.
extern GTSAM_EXPORT const PiecewisePolynomial<3, 3> IrwinHallCDF2;
/// Quartic cardinal-spline CDF kernel.
extern GTSAM_EXPORT const PiecewisePolynomial<4, 4> IrwinHallCDF3;
/// Quintic cardinal-spline CDF kernel.
extern GTSAM_EXPORT const PiecewisePolynomial<5, 5> IrwinHallCDF4;
/// Sixth-degree cardinal-spline CDF kernel.
extern GTSAM_EXPORT const PiecewisePolynomial<6, 6> IrwinHallCDF5;
/// Seventh-degree cardinal-spline CDF kernel.
extern GTSAM_EXPORT const PiecewisePolynomial<7, 7> IrwinHallCDF6;

}  // namespace kernels
}  // namespace gtsam
