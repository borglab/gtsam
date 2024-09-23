/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DoglegOptimizerImpl.h
 * @brief   Nonlinear factor graph optimizer using Powell's Dogleg algorithm (detail implementation)
 * @author  Richard Roberts
 */

#include <cmath>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

using namespace std;

namespace gtsam {
/* ************************************************************************* */
VectorValues DoglegOptimizerImpl::ComputeDoglegPoint(
    double delta, const VectorValues& dx_u, const VectorValues& dx_n, const bool verbose) {

  // Get magnitude of each update and find out which segment delta falls in
  assert(delta >= 0.0);
  double deltaSq = delta*delta;
  double x_u_norm_sq = dx_u.squaredNorm();
  double x_n_norm_sq = dx_n.squaredNorm();
  if(verbose) cout << "Steepest descent magnitude " << std::sqrt(x_u_norm_sq) << ", Newton's method magnitude " << std::sqrt(x_n_norm_sq) << endl;
  if(deltaSq < x_u_norm_sq) {
    // Trust region is smaller than steepest descent update
    VectorValues x_d = std::sqrt(deltaSq / x_u_norm_sq) * dx_u;
    if(verbose) cout << "In steepest descent region with fraction " << std::sqrt(deltaSq / x_u_norm_sq) << " of steepest descent magnitude" << endl;
    return x_d;
  } else if(deltaSq < x_n_norm_sq) {
    // Trust region boundary is between steepest descent point and Newton's method point
    return ComputeBlend(delta, dx_u, dx_n, verbose);
  } else {
    assert(deltaSq >= x_n_norm_sq);
    if(verbose) cout << "In pure Newton's method region" << endl;
    // Trust region is larger than Newton's method point
    return dx_n;
  }
}

/* ************************************************************************* */
VectorValues DoglegOptimizerImpl::ComputeBlend(double delta, const VectorValues& x_u, const VectorValues& x_n, const bool verbose) {

  // See doc/trustregion.lyx or doc/trustregion.pdf

  // Compute inner products
  const double un = dot(x_u, x_n);
  const double uu = dot(x_u, x_u);
  const double nn = dot(x_n, x_n);

  // Compute quadratic formula terms
  const double a = uu - 2.*un + nn;
  const double b = 2. * (un - uu);
  const double c = uu - delta*delta;
  double sqrt_b_m4ac = std::sqrt(b*b - 4*a*c);

  // Compute blending parameter
  double tau1 = (-b + sqrt_b_m4ac) / (2.*a);
  double tau2 = (-b - sqrt_b_m4ac) / (2.*a);

  double tau;
  if(0.0 <= tau1 && tau1 <= 1.0) {
    assert(!(0.0 <= tau2 && tau2 <= 1.0));
    tau = tau1;
  } else {
    assert(0.0 <= tau2 && tau2 <= 1.0);
    tau = tau2;
  }

  // Compute blended point
  if(verbose) cout << "In blend region with fraction " << tau << " of Newton's method point" << endl;
  VectorValues blend = (1. - tau) * x_u;
  blend += tau * x_n;
  return blend;
}

}
