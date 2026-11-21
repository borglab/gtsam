#pragma once
#include <gtsam/base/GroupAction.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/TangentLieGroup.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Rot3.h>

#include <Eigen/Dense>

namespace gtsam {
namespace tgeqf {

/// Tangent group of SE_2(3) under its adjoint action: pairs (A, a) with
/// product (A1,a1)(A2,a2) = (A1 A2, a1 + Ad_{A1} a2).
using TGElement = gtsam::TangentLieGroup<gtsam::ExtendedPose3<2>>;

/// Biased INS manifold state xi = (T, b), T = [R, v, p] in SE_2(3) and
/// b = [b_w; b_a; b_v] in R^9 (dim 18).
struct State {
  gtsam::ExtendedPose3<2> T;
  Vector9 b = Vector9::Zero();

  static constexpr int dimension = 18;

  /// Tangent vector of the state manifold, in the origin chart.
  using TangentVector = Eigen::Matrix<double, 18, 1>;
  /// Jacobian of a map between tangent spaces of the state manifold.
  using Jacobian = Eigen::Matrix<double, 18, 18>;

  static State identity() { return {}; }

  Matrix5 T_matrix() const { return T.matrix(); }

  /// Retract in the origin chart: Retract(xi, eps) = phi(Expmap_G(eps), xi).
  /// The navigation block reads SE_2(3) logarithm coordinates in T's body
  /// frame; the bias block goes through TGElement's fiber exponential rather
  /// than a plain vector difference.
  State retract(const TangentVector& delta) const;

  /// Local coordinates in the origin chart; inverse of retract.
  TangentVector localCoordinates(const State& other) const;
};

}  // namespace tgeqf
}  // namespace gtsam

namespace gtsam {

template <>
struct traits<tgeqf::State> {
  using ManifoldType = tgeqf::State;
  using TangentVector = tgeqf::State::TangentVector;
  using Jacobian = tgeqf::State::Jacobian;
  using structure_category = manifold_tag;

  static constexpr int dimension = 18;
  static int GetDimension(const tgeqf::State&) { return dimension; }

  static tgeqf::State Retract(const tgeqf::State& xi,
                              const TangentVector& delta) {
    return xi.retract(delta);
  }

  /// Inverse of Retract.
  static TangentVector Local(const tgeqf::State& xi_ref,
                             const tgeqf::State& xi_est) {
    return xi_ref.localCoordinates(xi_est);
  }

  static tgeqf::State Identity() { return tgeqf::State::identity(); }

  static bool Equals(const tgeqf::State& a, const tgeqf::State& b,
                     double tol = 1e-9);

  static void Print(const tgeqf::State& xi, const std::string& str = "");
};

template <>
struct traits<const tgeqf::State> : traits<tgeqf::State> {};

template <>
struct traits<const tgeqf::TGElement> : traits<tgeqf::TGElement> {};

}  // namespace gtsam

namespace gtsam {
namespace tgeqf {

/// Right group action phi : G x M -> M for TG-EqF.
struct TGSymmetry {
  using Group = TGElement;
  using Manifold = State;

  /// phi(X, phi(Y, xi)) == phi(Y * X, xi), so this is a right action.
  static constexpr ActionType type = ActionType::Right;

  /// Maps G -> M by acting on a fixed reference state (orbit functor).
  struct Orbit {
    State xi_ref;

    explicit Orbit(const State& xi_ref);

    /// phi(X, xi_ref) with optional Jacobian d(phi)/dX in R^{18 x 18}.
    State operator()(const TGElement& X, State::Jacobian* H = nullptr) const;
  };

  /// Maps M -> M by acting with a fixed X (diffeomorphism functor).
  struct Diffeomorphism {
    TGElement X;

    explicit Diffeomorphism(const TGElement& X);

    /// phi(X, xi) with optional Jacobian d(phi)/dxi in R^{18 x 18}.
    State operator()(const State& xi, State::Jacobian* H = nullptr) const;
  };
};

/// Standalone right group action phi(X, xi).
State phi(const TGElement& X, const State& xi);

/// The unique X with phi(X, xi_ref) == xi_est. Initializes an EqF whose
/// origin is fixed at xi_ref but whose estimate starts at xi_est.
TGElement phiInverse(const State& xi_ref, const State& xi_est);

}  // namespace tgeqf
}  // namespace gtsam
