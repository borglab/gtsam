/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MEKFGeometry.h
 * @brief MEKF state manifold SO(3) x R^12 and its retraction.
 * @author Arihant Lunawat
 *
 * MEKFState: xi = [R, v, p, b_gyro, b_accel] on SO(3) x R^12 (dim 15).
 *
 * Error convention (body/right multiplicative, additive on vectors):
 *     R_true = R_hat * Exp(d_theta),  v = v_hat + d_v, ...
 * Matches TG-EqF and TFG-InEKF so NEES/error metrics are comparable.
 * Tangent block order: R, v, p, b_gyro, b_accel.
 */

#pragma once
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam_unstable/dllexport.h>

#include <Eigen/Dense>

namespace gtsam {
namespace mekf {

struct GTSAM_UNSTABLE_EXPORT MEKFState {
  Rot3 R;
  Vector3 v;
  Vector3 p;
  Vector3 b_gyro;
  Vector3 b_accel;

  static constexpr int dimension = 15;

  using TangentVector = Eigen::Matrix<double, 15, 1>;
  using Jacobian = Eigen::Matrix<double, 15, 15>;

  MEKFState();
  MEKFState(const Rot3& R, const Vector3& v, const Vector3& p,
            const Vector3& b_gyro, const Vector3& b_accel);

  static MEKFState identity();
};

}  // namespace mekf
}  // namespace gtsam

// GTSAM Manifold traits for ManifoldEKF<MEKFState>.
namespace gtsam {

template <>
struct GTSAM_UNSTABLE_EXPORT traits<mekf::MEKFState> {
  using ManifoldType = mekf::MEKFState;
  using TangentVector = mekf::MEKFState::TangentVector;
  using Jacobian = mekf::MEKFState::Jacobian;
  using structure_category = manifold_tag;

  static constexpr int dimension = 15;
  static int GetDimension(const mekf::MEKFState&) { return dimension; }

  static mekf::MEKFState Retract(const mekf::MEKFState& xi,
                                 const TangentVector& delta);
  static TangentVector Local(const mekf::MEKFState& xi_ref,
                             const mekf::MEKFState& xi);

  static mekf::MEKFState Identity() { return mekf::MEKFState::identity(); }

  static bool Equals(const mekf::MEKFState& a, const mekf::MEKFState& b,
                     double tol = 1e-9);
  static void Print(const mekf::MEKFState& xi, const std::string& s = "");
};

template <>
struct traits<const mekf::MEKFState> : traits<mekf::MEKFState> {};

}  // namespace gtsam
