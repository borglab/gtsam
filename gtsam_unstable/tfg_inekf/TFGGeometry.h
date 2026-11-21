#pragma once
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>

#include <Eigen/Dense>

// Two-Frames Group G_TF = SO(3) x R^12 (dim 15), and its natural right
// action on itself (the state space of the TFG-InEKF).

namespace tfg {

using gtsam::Rot3;
using gtsam::Vector3;

/// Lie algebra g_TF as a 15-vector.
using Tangent = gtsam::Vector15;

class TwoFrameGroup {
 public:
  static constexpr int dim = 15;
  using TangentVector = Tangent;
  using Jacobian = Eigen::Matrix<double, dim, dim>;

  Rot3 R;
  Vector3 v;
  Vector3 p;
  Vector3 gamma_omega;
  Vector3 gamma_accel;

  TwoFrameGroup();

  TwoFrameGroup(const Rot3& R, const Vector3& v, const Vector3& p,
                const Vector3& gamma_omega, const Vector3& gamma_accel);

  /// Build from physical state (T, b); gamma = -R * b.
  static TwoFrameGroup FromState(const Rot3& R, const Vector3& v,
                                 const Vector3& p, const Vector3& b_omega,
                                 const Vector3& b_accel);

  /// Physical biases b = R^{-1} (-gamma).
  Vector3 bias_omega() const;
  Vector3 bias_accel() const;

  static TwoFrameGroup Identity();
  TwoFrameGroup operator*(const TwoFrameGroup& Y) const;
  TwoFrameGroup inverse() const;

  static TwoFrameGroup Expmap(const TangentVector& xi);
  TangentVector Logmap() const;

  /// Adjoint Ad_X (15x15) for InvariantEKF covariance propagation.
  Jacobian AdjointMap() const;

  /// 5x5 matrix realisation of the SE_2(3) navigation part.
  gtsam::Matrix5 C_matrix() const;
};

}  // namespace tfg

namespace gtsam {

template <>
struct traits<tfg::TwoFrameGroup> {
  using ManifoldType = tfg::TwoFrameGroup;
  using TangentVector = tfg::Tangent;
  using Jacobian = tfg::TwoFrameGroup::Jacobian;
  using ChartJacobian = OptionalJacobian<15, 15>;
  using structure_category = lie_group_tag;
  using group_flavor = multiplicative_group_tag;

  static constexpr int dimension = 15;
  static int GetDimension(const tfg::TwoFrameGroup&) { return dimension; }

  // Testable
  static bool Equals(const tfg::TwoFrameGroup& X, const tfg::TwoFrameGroup& Y,
                     double tol = 1e-9);
  static void Print(const tfg::TwoFrameGroup& X, const std::string& s = "");

  // Group
  static tfg::TwoFrameGroup Identity();
  static tfg::TwoFrameGroup Compose(const tfg::TwoFrameGroup& X,
                                    const tfg::TwoFrameGroup& Y,
                                    ChartJacobian Hx = {},
                                    ChartJacobian Hy = {});
  static tfg::TwoFrameGroup Between(const tfg::TwoFrameGroup& X,
                                    const tfg::TwoFrameGroup& Y,
                                    ChartJacobian Hx = {},
                                    ChartJacobian Hy = {});
  static tfg::TwoFrameGroup Inverse(const tfg::TwoFrameGroup& X,
                                    ChartJacobian H = {});

  // Lie group
  static tfg::TwoFrameGroup Expmap(const TangentVector& xi,
                                   ChartJacobian H = {});
  static TangentVector Logmap(const tfg::TwoFrameGroup& X,
                              ChartJacobian H = {});
  static Jacobian AdjointMap(const tfg::TwoFrameGroup& X);

  // Manifold
  static tfg::TwoFrameGroup Retract(const tfg::TwoFrameGroup& X,
                                    const TangentVector& xi);
  static TangentVector Local(const tfg::TwoFrameGroup& X,
                             const tfg::TwoFrameGroup& Y);
};

template <>
struct traits<const tfg::TwoFrameGroup> : traits<tfg::TwoFrameGroup> {};

}  // namespace gtsam

namespace tfg {

/// Right group action phi(X, xi) = xi * X. The InEKF operates directly on
/// TwoFrameGroup, so this is only needed for equivariance checks.
TwoFrameGroup phi(const TwoFrameGroup& X, const TwoFrameGroup& xi);

}  // namespace tfg
