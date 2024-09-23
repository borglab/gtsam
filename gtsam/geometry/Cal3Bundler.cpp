/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Bundler.cpp
 * @date Sep 25, 2010
 * @author ydjian
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

/* ************************************************************************* */
Matrix3 Cal3Bundler::K() const {
  // This function is needed to ensure skew = 0;
  Matrix3 K;
  K << fx_, 0, u0_, 0, fy_, v0_, 0, 0, 1.0;
  return K;
}

/* ************************************************************************* */
Vector4 Cal3Bundler::k() const {
  Vector4 rvalue_;
  rvalue_ << k1_, k2_, 0, 0;
  return rvalue_;
}

/* ************************************************************************* */
Vector3 Cal3Bundler::vector() const { return Vector3(fx_, k1_, k2_); }

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3Bundler& cal) {
  os << "f: " << cal.fx() << ", k1: " << cal.k1() << ", k2: " << cal.k2()
     << ", px: " << cal.px() << ", py: " << cal.py();
  return os;
}

/* ************************************************************************* */
void Cal3Bundler::print(const std::string& s) const {
  gtsam::print((Vector)(Vector(5) << fx_, k1_, k2_, u0_, v0_).finished(),
               s + ".K");
}

/* ************************************************************************* */
bool Cal3Bundler::equals(const Cal3Bundler& K, double tol) const {
  const Cal3* base = dynamic_cast<const Cal3*>(&K);
  return (Cal3::equals(*base, tol) && std::fabs(k1_ - K.k1_) < tol &&
          std::fabs(k2_ - K.k2_) < tol && std::fabs(u0_ - K.u0_) < tol &&
          std::fabs(v0_ - K.v0_) < tol);
}

/* ************************************************************************* */
Point2 Cal3Bundler::uncalibrate(const Point2& p, OptionalJacobian<2, 3> Dcal,
                                OptionalJacobian<2, 2> Dp) const {
  //  r = x² + y²;
  //  g = (1 + k(1)*r + k(2)*r²);
  //  pi(:,i) = g * pn(:,i)
  const double x = p.x(), y = p.y();
  const double r = x * x + y * y;
  const double g = 1. + (k1_ + k2_ * r) * r;
  const double u = g * x, v = g * y;

  const double f_ = fx_;

  // Derivatives make use of intermediate variables above
  if (Dcal) {
    double rx = r * x, ry = r * y;
    *Dcal << u, f_ * rx, f_ * r * rx, v, f_ * ry, f_ * r * ry;
  }

  if (Dp) {
    const double a = 2. * (k1_ + 2. * k2_ * r);
    const double axx = a * x * x, axy = a * x * y, ayy = a * y * y;
    *Dp << g + axx, axy, axy, g + ayy;
    *Dp *= f_;
  }

  return Point2(u0_ + f_ * u, v0_ + f_ * v);
}

/* ************************************************************************* */
Point2 Cal3Bundler::calibrate(const Point2& pi, OptionalJacobian<2, 3> Dcal,
                              OptionalJacobian<2, 2> Dp) const {
  // Copied from Cal3DS2
  // but specialized with k1, k2 non-zero only and fx=fy and s=0
  double px = (pi.x() - u0_) / fx_, py = (pi.y() - v0_) / fx_;
  const Point2 invKPi(px, py);
  Point2 pn;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration = 0;
  do {
    // initialize pn with distortion included
    const double rr = (px * px) + (py * py);
    const double g = (1 + k1_ * rr + k2_ * rr * rr);
    pn = invKPi / g;

    if (distance2(uncalibrate(pn), pi) <= tol_) break;

    // Set px and py using intrinsic coordinates since that is where radial
    // distortion correction is done.
    px = pn.x();
    py = pn.y();
    iteration++;

  } while (iteration < maxIterations);

  if (iteration >= maxIterations)
    throw std::runtime_error(
        "Cal3Bundler::calibrate fails to converge. need a better "
        "initialization");

  calibrateJacobians<Cal3Bundler, dimension>(*this, pn, Dcal, Dp);

  return pn;
}

/* ************************************************************************* */
Matrix2 Cal3Bundler::D2d_intrinsic(const Point2& p) const {
  Matrix2 Dp;
  uncalibrate(p, {}, Dp);
  return Dp;
}

/* ************************************************************************* */
Matrix23 Cal3Bundler::D2d_calibration(const Point2& p) const {
  Matrix23 Dcal;
  uncalibrate(p, Dcal, {});
  return Dcal;
}

/* ************************************************************************* */
Matrix25 Cal3Bundler::D2d_intrinsic_calibration(const Point2& p) const {
  Matrix23 Dcal;
  Matrix2 Dp;
  uncalibrate(p, Dcal, Dp);
  Matrix25 H;
  H << Dp, Dcal;
  return H;
}

}  // namespace gtsam
