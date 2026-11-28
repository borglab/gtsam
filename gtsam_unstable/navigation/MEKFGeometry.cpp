/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MEKFGeometry.cpp
 * @brief MEKF state construction and manifold traits.
 * @author Arihant Lunawat
 */

#include <gtsam_unstable/navigation/MEKFGeometry.h>

#include <iostream>

namespace gtsam {
namespace mekf {

MEKFState::MEKFState()
    : v(Vector3::Zero()),
      p(Vector3::Zero()),
      b_gyro(Vector3::Zero()),
      b_accel(Vector3::Zero()) {}

MEKFState::MEKFState(const Rot3& R, const Vector3& v, const Vector3& p,
                     const Vector3& b_gyro, const Vector3& b_accel)
    : R(R), v(v), p(p), b_gyro(b_gyro), b_accel(b_accel) {}

MEKFState MEKFState::identity() { return MEKFState(); }

}  // namespace mekf

mekf::MEKFState traits<mekf::MEKFState>::Retract(const mekf::MEKFState& xi,
                                                 const TangentVector& delta) {
  mekf::MEKFState result;
  result.R = xi.R * Rot3::Expmap(delta.segment<3>(0));
  result.v = xi.v + delta.segment<3>(3);
  result.p = xi.p + delta.segment<3>(6);
  result.b_gyro = xi.b_gyro + delta.segment<3>(9);
  result.b_accel = xi.b_accel + delta.segment<3>(12);
  return result;
}

traits<mekf::MEKFState>::TangentVector traits<mekf::MEKFState>::Local(
    const mekf::MEKFState& xi_ref, const mekf::MEKFState& xi) {
  TangentVector delta;
  delta.segment<3>(0) = Rot3::Logmap(xi_ref.R.inverse() * xi.R);
  delta.segment<3>(3) = xi.v - xi_ref.v;
  delta.segment<3>(6) = xi.p - xi_ref.p;
  delta.segment<3>(9) = xi.b_gyro - xi_ref.b_gyro;
  delta.segment<3>(12) = xi.b_accel - xi_ref.b_accel;
  return delta;
}

bool traits<mekf::MEKFState>::Equals(const mekf::MEKFState& a,
                                     const mekf::MEKFState& b, double tol) {
  return a.R.equals(b.R, tol) && (a.v - b.v).norm() < tol &&
         (a.p - b.p).norm() < tol && (a.b_gyro - b.b_gyro).norm() < tol &&
         (a.b_accel - b.b_accel).norm() < tol;
}

void traits<mekf::MEKFState>::Print(const mekf::MEKFState& xi,
                                    const std::string& s) {
  std::cout << s;
  xi.R.print("  R: ");
  std::cout << "  v:       " << xi.v.transpose() << "\n"
            << "  p:       " << xi.p.transpose() << "\n"
            << "  b_gyro:  " << xi.b_gyro.transpose() << "\n"
            << "  b_accel: " << xi.b_accel.transpose() << "\n";
}

}  // namespace gtsam
