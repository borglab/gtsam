
#pragma once

#include <gtsam/base/GroupAction.h>
#include <gtsam_unstable/navigation/GalileanCommon.h>

namespace gtsam {
namespace galilean {

/**
 * Right action φ(ξ, X) = (F·A, Ad_{A⁻¹}(b - a)) of G on M (eq. 28).
 */
struct GTSAM_UNSTABLE_EXPORT Symmetry
    : public GroupAction<Symmetry, Group, State> {
  static constexpr ActionType type = ActionType::Right;

  /**
   * @brief Evaluate the right action φ(ξ, X).
   * @param xi State argument.
   * @param X Group argument.
   * @param H_xi Optional Jacobian w.r.t. the state.
   * @param H_X Optional Jacobian w.r.t. the group element.
   */
  State operator()(const State& xi, const Group& X,
                   OptionalJacobian<20, 20> H_xi = {},
                   OptionalJacobian<20, 20> H_X = {}) const;
};

/**
 * Exact discrete system (eq. 25), noise-free:
 *   F⁺ = exp(-g_N^ dt) F exp((w_N - b)^ dt),   b⁺ = b.
 */
GTSAM_UNSTABLE_EXPORT State Dynamics(const State& xi, const IMUInput& imu,
                                     const Vector10& gN, double dt);

/**
 * Discrete lift Λ(ξ, u) (eqs. 29-31) with zero bias random walk:
 *   Λ_F = exp(-Ad_{F⁻¹} g_N^ dt) exp((w_N - b)^ dt),
 *   Λ_b = b - Ad_{Λ_F} b.
 * Satisfies φ(ξ, Λ(ξ, u)) = Dynamics(ξ, u). It is a product of exponentials,
 * so it is passed to EquivariantFilter::predictWithStep() as a group element.
 */
GTSAM_UNSTABLE_EXPORT Group LiftDiscrete(const State& xi, const IMUInput& imu,
                                         const Vector10& gN, double dt);

/// @name EqF linearization (GTSAM error coordinates)
/// @{

/**
 * Discrete error transition Φ (eqs. 34-35).
 * @param X Current group estimate.
 * @param gN Earth input.
 * @param wTilde Bias-corrected input w_N - b̂.
 * @param dt Time step.
 */
GTSAM_UNSTABLE_EXPORT Matrix20 EqFStateMatrixA(const Group& X,
                                               const Vector10& gN,
                                               const Vector10& wTilde,
                                               double dt);

/**
 * Discrete input matrix B (eq. 36). Input noise ordering:
 * [gyro(3), acc(3), ν virtual(3), ρ virtual(1), bias walks(10)].
 * Qd = B Q Bᵀ / dt with Q the continuous-time input noise.
 */
GTSAM_UNSTABLE_EXPORT Matrix20 EqFInputMatrixB(const Group& X,
                                               const Vector10& gN,
                                               const Vector10& wTilde,
                                               double dt);

/**
 * Delayed GNSS output (eqs. 37-38):  y = R_T p₀ + p_T,  T = F Υ⁻¹.
 * @param F Galilean frame; its time component is the delay δ.
 * @param Upsilon Preintegration Υ(δ) from t-δ to now.
 * @param wStart Input w̃ held at t-δ (∂Υ/∂δ); zero to treat Υ as fixed.
 * @param leverArm Antenna position p₀ in the IMU frame.
 * @param H Optional Jacobian w.r.t. F (right perturbation).
 */
GTSAM_UNSTABLE_EXPORT Point3 GnssOutput(const Gal3& F, const Gal3& Upsilon,
                                        const Vector10& wStart,
                                        const Point3& leverArm,
                                        OptionalJacobian<3, 10> H = {});

/**
 * Output matrix C in error coordinates (eqs. 40-42): C = [H_F, 0] · Dφ_X.
 * @param X Current group estimate.
 * @param H_F Jacobian of the output w.r.t. F at the current estimate.
 */
GTSAM_UNSTABLE_EXPORT Matrix3x20 EqFOutputMatrixC(const Group& X,
                                                  const Matrix3x10& H_F);

/// Lift a correction from error coordinates to the group tangent: S δξ.
GTSAM_UNSTABLE_EXPORT Vector20 LiftInnovation(const Vector20& deltaXi);

/**
 * Covariance reset Jacobian (Alg. 1, update ⋆⋆): linearization of the chart
 * change e' = φ(e, exp(-δx)) at the posterior error mean ε̄.
 */
GTSAM_UNSTABLE_EXPORT Matrix20 ResetJacobian(const Vector20& epsilonBar,
                                             const Vector20& deltaX);
/// @}

}  // namespace galilean
}  // namespace gtsam
