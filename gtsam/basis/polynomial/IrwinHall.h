#pragma once

#include "kernels.h"

namespace gtsam {
namespace kernels {

/*
Irwin Hall coefficients provide canonical spline basis functions
These functions are piecewise-defined polynomials with continuity up to their (N-1)th derivative
Irwin Hall 0 is a zeroth order spline (a discontinuous staircase)
Irwin Hall 3 is a cubic polynomial, so it has continuous (piecewise linear) accelerations

https://oeis.org/A188816

Mathematica:
f[n_, k_] := 
  f[n, k] = Sum[(-1)^j  Binomial[n, j]  (x - j)^(n - 1), {j, 0, k}];
T[n_, k_] := Table[Coefficient[f[n, k], x, t], {t, 0, n - 1}];
Table[Table[T[n, k]/((n - 1)!), {k, 0, n - 1}], {n, 1, 7}]


This series of fucntions converges pretty quickly to a gaussian.
If you need higher orders (N), you can use a gaussian kernel directly
e^(-(1/2)((x-N/2)/(0.3*N/2)^2 )
or defined in the fourier domain, cardinal splines are defined as sinc^N, and gaussians remain gaussian:
  'sinc to the n' has the same form as a gaussian with a simple domain transformation
  e^(-0.5(w^2) * N/3) ~= sinc^N(w)

the derivatives of a gaussian depend on Hermite polynomials which are a little complicated

The inverse filter (1/sinc^N) for this explodes pretty fast at higher frequencies,
  so filters beyond order 4 should step back a long way (up to an order of magnitude) from the nyquist limit
  in order to avoid the vector sum crossing the edge of the Lie Algebra's Map

High frequencies can be attenuated by processing the control points under a sharp
  high-pass filter and passing factors that bais the result to zero.
  The high-pass factors form a 'penalty' term under a generalisation over P-Splines.


*/
extern const piecewise_polynomial<1,1> IrwinHall0;
extern const piecewise_polynomial<2,2> IrwinHall1;
extern const piecewise_polynomial<3,3> IrwinHall2;
extern const piecewise_polynomial<4,4> IrwinHall3; // equivalent to cubic spline
extern const piecewise_polynomial<5,5> IrwinHall4;
extern const piecewise_polynomial<6,6> IrwinHall5;
extern const piecewise_polynomial<7,7> IrwinHall6;

} // namespace kernels
} // namespace gtsam