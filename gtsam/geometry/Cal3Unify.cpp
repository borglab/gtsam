/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Unify.cpp
 * @date Mar 8, 2014
 * @author Jing Dong
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3Unify.h>

#include <cmath>

namespace gtsam {

/* ************************************************************************* */
Cal3Unify::Cal3Unify(const Vector &v):
    Cal3DS2(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]), xi_(v[9]) {}

/* ************************************************************************* */
Matrix Cal3Unify::K() const {
  return Base::K();
}

/* ************************************************************************* */
Vector Cal3Unify::vector() const {
  return (Vector(10) << Base::vector(), xi_);
}

/* ************************************************************************* */
void Cal3Unify::print(const std::string& s) const {
  Base::print(s);
  gtsam::print((Vector)(Vector(1) << xi_), s + ".xi");
}

/* ************************************************************************* */
bool Cal3Unify::equals(const Cal3Unify& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol || fabs(s_ - K.s_) > tol ||
      fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol || fabs(k1_ - K.k1_) > tol ||
      fabs(k2_ - K.k2_) > tol || fabs(k3_ - K.k3_) > tol || fabs(k4_ - K.k4_) > tol ||
      fabs(xi_ - K.xi_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
Point2 Cal3Unify::uncalibrate(const Point2& p,
       boost::optional<Matrix&> H1,
       boost::optional<Matrix&> H2) const {

  // this part of code is modified from Cal3DS2,
  // since the second part of this model (after project to normalized plane)
  // is same as Cal3DS2

  // parameters
  const double xi = xi_;

  // Part1: project 3D space to NPlane
  const double xs = p.x(), ys = p.y();  // normalized points in 3D space
  const double sqrt_nx = sqrt(xs * xs + ys * ys + 1.0);
  const double xi_sqrt_nx = 1.0 / (1 + xi * sqrt_nx);
  const double xi_sqrt_nx2 = xi_sqrt_nx * xi_sqrt_nx;
  const double x = xs * xi_sqrt_nx, y = ys * xi_sqrt_nx; // points on NPlane

  // Part2: project NPlane point to pixel plane: use Cal3DS2
  Point2 m(x,y);
  Matrix H1base, H2base;    // jacobians from Base class
  Point2 puncalib = Base::uncalibrate(m, H1base, H2base);

  // Inlined derivative for calibration
  if (H1) {
    // part1
    Matrix DU = (Matrix(2,1) << -xs * sqrt_nx * xi_sqrt_nx2,
        -ys * sqrt_nx * xi_sqrt_nx2);
    Matrix DDS2U = H2base * DU;

    *H1 = collect(2, &H1base, &DDS2U);
  }
  // Inlined derivative for points
  if (H2) {
    // part1
    const double denom = 1.0 * xi_sqrt_nx2 / sqrt_nx;
    const double mid = -(xi * xs*ys) * denom;
    Matrix DU = (Matrix(2, 2) <<
        (sqrt_nx + xi*(ys*ys + 1)) * denom, mid,
        mid, (sqrt_nx + xi*(xs*xs + 1)) * denom);

    *H2 = H2base * DU;
  }

  return puncalib;
}

/* ************************************************************************* */
Point2 Cal3Unify::calibrate(const Point2& pi, const double tol) const {
  // Use the following fixed point iteration to invert the radial distortion.
  // pn_{t+1} = (inv(K)*pi - dp(pn_{t})) / g(pn_{t})

  // point on the normalized plane, input for DS2
  double px = pi.x();
  double py = pi.y();
  const Point2 invKPi ((1 / fx_) * (px - u0_ - (s_ / fy_) * (py - v0_)),
                       (1 / fy_) * (py - v0_));

  // initialize by ignoring the distortion at all, might be problematic for pixels around boundary
  Point2 pn = nPlaneToSpace(invKPi);

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 20;
  int iteration;
  for ( iteration = 0; iteration < maxIterations; ++iteration ) {

    if ( uncalibrate(pn).distance(pi) <= tol ) break;

    // part1: 3D space -> normalized plane
    Point2 pnpl = spaceToNPlane(pn);
    // part2: normalized plane -> 3D space
    px = pnpl.x(), py = pnpl.y();
    const double xy = px*py, xx = px*px, yy = py*py;
    const double rr = xx + yy;
    const double g = (1+k1_*rr+k2_*rr*rr);
    const double dx = 2*k3_*xy + k4_*(rr+2*xx);
    const double dy = 2*k4_*xy + k3_*(rr+2*yy);
    pn = nPlaneToSpace((invKPi - Point2(dx,dy))/g);
  }

  if ( iteration >= maxIterations )
    throw std::runtime_error("Cal3Unify::calibrate fails to converge. need a better initialization");

  return pn;
}
/* ************************************************************************* */
Point2 Cal3Unify::nPlaneToSpace(const Point2& p) const {

  const double x = p.x(), y = p.y();
  const double xy2 = x * x + y * y;
  const double sq_xy = (xi_ + sqrt(1 + (1 - xi_ * xi_) * xy2)) / (xy2 + 1);

  return Point2((sq_xy * x / (sq_xy - xi_)), (sq_xy * y / (sq_xy - xi_)));
}

/* ************************************************************************* */
Point2 Cal3Unify::spaceToNPlane(const Point2& p) const {

  const double x = p.x(), y = p.y();
  const double sq_xy = 1 + xi_ * sqrt(x * x + y * y + 1);

  return Point2((x / sq_xy), (y / sq_xy));
}

/* ************************************************************************* */
Cal3Unify Cal3Unify::retract(const Vector& d) const {
  return Cal3Unify(vector() + d);
}

/* ************************************************************************* */
Vector Cal3Unify::localCoordinates(const Cal3Unify& T2) const {
  return T2.vector() - vector();
}

}
/* ************************************************************************* */


