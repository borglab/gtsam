#pragma once

#include "PiecewisePolynomial.h"

namespace gtsam {
namespace kernels {

/*
Irwin Hall coefficients are used by TrajectoryModel to implement
  canonical (monotonic) spline basis functions.
These functions are piecewise-defined polynomials continuous up to the (N-1)th derivative
Irwin Hall CDF 0 is a zeroth order spline (linear interpolation)
Irwin Hall CDF 2 is a cubic polynomial, so it has linear accelerations

This series of fucntions converges pretty quickly to a gaussian.
If you need higher orders (N), you can use a gaussian kernel and CDF directly
e^(-(1/2)((x-N/2)/(0.3*N/2)^2 )
*/

// https://oeis.org/A188816
extern const piecewise_polynomial<0,1> IrwinHall0;
extern const piecewise_polynomial<1,2> IrwinHall1;
extern const piecewise_polynomial<2,3> IrwinHall2;
extern const piecewise_polynomial<3,4> IrwinHall3;
extern const piecewise_polynomial<4,5> IrwinHall4;
extern const piecewise_polynomial<5,6> IrwinHall5;
extern const piecewise_polynomial<6,7> IrwinHall6;

// https://oeis.org/A188668
extern const piecewise_polynomial<1,1> IrwinHallCDF0; // produces a linear interpolator
extern const piecewise_polynomial<2,2> IrwinHallCDF1;
extern const piecewise_polynomial<3,3> IrwinHallCDF2; // produces a cubic spline
extern const piecewise_polynomial<4,4> IrwinHallCDF3;
extern const piecewise_polynomial<5,5> IrwinHallCDF4;
extern const piecewise_polynomial<6,6> IrwinHallCDF5;
extern const piecewise_polynomial<7,7> IrwinHallCDF6;

} // namespace kernels
} // namespace gtsam