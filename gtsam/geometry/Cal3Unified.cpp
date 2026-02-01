/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Unified.cpp
 * @date Mar 8, 2014
 * @author Jing Dong
 * @author Varun Agrawal
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Point2.h>

#include <cmath>

namespace gtsam {

/* ************************************************************************* */
Vector10 Cal3Unified::vector() const {
  Vector10 v;
  v << Base::vector(), xi_;
  return v;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3Unified& cal) {
  os << (Cal3DS2_Base&)cal;
  os << ", xi: " << cal.xi();
  return os;
}

/* ************************************************************************* */
void Cal3Unified::print(const std::string& s) const {
  Base::print(s);
  gtsam::print((Vector)(Vector(1) << xi_).finished(), s + ".xi");
}

/* ************************************************************************* */
bool Cal3Unified::equals(const Cal3Unified& K, double tol) const {
  const Cal3DS2_Base* base = dynamic_cast<const Cal3DS2_Base*>(&K);
  return Cal3DS2_Base::equals(*base, tol) && std::fabs(xi_ - K.xi_) < tol;
}

/* ************************************************************************* */
// todo: make a fixed sized jacobian version of this
Point2 Cal3Unified::uncalibrate(const Point2& p, OptionalJacobian<2, 10> Dcal,
                                OptionalJacobian<2, 2> Dp) const {
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
  const double x = xs * xi_sqrt_nx, y = ys * xi_sqrt_nx;  // points on NPlane

  // Part2: project NPlane point to pixel plane: use Cal3DS2
  Point2 m(x, y);
  Matrix29 H1base;
  Matrix2 H2base;  // jacobians from Base class
  Point2 puncalib = Base::uncalibrate(m, H1base, H2base);

  // Inlined derivative for calibration
  if (Dcal) {
    // part1
    Vector2 DU;
    DU << -xs * sqrt_nx * xi_sqrt_nx2,  //
        -ys * sqrt_nx * xi_sqrt_nx2;
    *Dcal << H1base, H2base * DU;
  }

  // Inlined derivative for points
  if (Dp) {
    // part1
    const double denom = 1.0 * xi_sqrt_nx2 / sqrt_nx;
    const double mid = -(xi * xs * ys) * denom;
    Matrix2 DU;
    DU << (sqrt_nx + xi * (ys * ys + 1)) * denom, mid,  //
        mid, (sqrt_nx + xi * (xs * xs + 1)) * denom;

    *Dp << H2base * DU;
  }

  return puncalib;
}

/* ************************************************************************* */
Point2 Cal3Unified::calibrate(const Point2& pi, OptionalJacobian<2, 10> Dcal,
                              OptionalJacobian<2, 2> Dp) const {
  // calibrate point to Nplane use base class::calibrate()
  Point2 pnplane = Base::calibrate(pi);

  // call nplane to space
  Point2 pn = this->nPlaneToSpace(pnplane);

  calibrateJacobians<Cal3Unified, dimension>(*this, pn, Dcal, Dp);

  return pn;
}
/* ************************************************************************* */
Point2 Cal3Unified::nPlaneToSpace(const Point2& p) const {
  const double x = p.x(), y = p.y();
  const double xy2 = x * x + y * y;
  const double sq_xy = (xi_ + sqrt(1 + (1 - xi_ * xi_) * xy2)) / (xy2 + 1);

  return Point2((sq_xy * x / (sq_xy - xi_)), (sq_xy * y / (sq_xy - xi_)));
}

/* ************************************************************************* */
Point2 Cal3Unified::spaceToNPlane(const Point2& p) const {
  const double x = p.x(), y = p.y();
  const double sq_xy = 1 + xi_ * sqrt(x * x + y * y + 1);

  return Point2((x / sq_xy), (y / sq_xy));
}

/* ************************************************************************* */
Cal3Unified Cal3Unified::retract(const Vector& d) const {
  return Cal3Unified(vector() + d);
}

/* ************************************************************************* */
Vector Cal3Unified::localCoordinates(const Cal3Unified& T2) const {
  return T2.vector() - vector();
}

/* ************************************************************************* */

}  // \ namespace gtsam
