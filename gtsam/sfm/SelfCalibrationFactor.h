/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2024, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file SelfCalibrationFactor.h
 * @brief Focal-length self-calibration factor from a fundamental matrix.
 * @author Kathirvel Gounder
 * @date June 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <array>
#include <cmath>

namespace gtsam {

/**
 * Build one 4-vector of bilinear coefficients from two pairs of 3-vectors,
 * picking rows (u, v) — the 2x2 minor of (ai/bi) against (aj/bj) used by the
 * Fetzer focal-length residual. Ported from GLOMAP (Lindenberger's DMAP).
 */
inline Eigen::Vector4d fetzer_d(const Eigen::Vector3d& ai,
                                const Eigen::Vector3d& bi,
                                const Eigen::Vector3d& aj,
                                const Eigen::Vector3d& bj, int u, int v) {
  Eigen::Vector4d d;
  d(0) = ai(u) * aj(v) - ai(v) * aj(u);
  d(1) = ai(u) * bj(v) - ai(v) * bj(u);
  d(2) = bi(u) * aj(v) - bi(v) * aj(u);
  d(3) = bi(u) * bj(v) - bi(v) * bj(u);
  return d;
}

/**
 * Precompute the Fetzer coefficient vectors from G = K_j^T F K_i.
 *
 * A single SVD of the (principal-point corrected, focal-free) matrix G, from
 * which the essential-matrix constraint becomes a rational equation in the two
 * focal lengths. Returns {d_01, d_02, d_12}; only d_01 and d_12 enter the
 * residual (d_02 is the redundant third constraint, kept for parity/testing).
 */
inline std::array<Eigen::Vector4d, 3> fetzer_ds(const Eigen::Matrix3d& G) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      G, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector3d s = svd.singularValues();

  const Eigen::Vector3d v_0 = svd.matrixV().col(0);
  const Eigen::Vector3d v_1 = svd.matrixV().col(1);
  const Eigen::Vector3d u_0 = svd.matrixU().col(0);
  const Eigen::Vector3d u_1 = svd.matrixU().col(1);

  const Eigen::Vector3d ai(
      s(0) * s(0) * (v_0(0) * v_0(0) + v_0(1) * v_0(1)),
      s(0) * s(1) * (v_0(0) * v_1(0) + v_0(1) * v_1(1)),
      s(1) * s(1) * (v_1(0) * v_1(0) + v_1(1) * v_1(1)));

  const Eigen::Vector3d aj(u_1(0) * u_1(0) + u_1(1) * u_1(1),
                           -(u_0(0) * u_1(0) + u_0(1) * u_1(1)),
                           u_0(0) * u_0(0) + u_0(1) * u_0(1));

  const Eigen::Vector3d bi(s(0) * s(0) * v_0(2) * v_0(2),
                           s(0) * s(1) * v_0(2) * v_1(2),
                           s(1) * s(1) * v_1(2) * v_1(2));

  const Eigen::Vector3d bj(u_1(2) * u_1(2), -(u_0(2) * u_1(2)),
                           u_0(2) * u_0(2));

  std::array<Eigen::Vector4d, 3> ds;
  ds[0] = fetzer_d(ai, bi, aj, bj, 1, 0);  // d_01
  ds[1] = fetzer_d(ai, bi, aj, bj, 0, 2);  // d_02 (unused in residual)
  ds[2] = fetzer_d(ai, bi, aj, bj, 2, 1);  // d_12
  return ds;
}

/**
 * @class SelfCalibrationFactor
 * @brief Binary factor measuring how close a fundamental matrix is to admitting
 * a valid essential matrix, as a function of the two cameras' focal lengths.
 *
 * Implements the focal-length self-calibration residual of Fetzer et al.
 * (WACV 2020). For an edge with fundamental matrix F and known principal points
 * pp_i, pp_j, a valid essential matrix E = K_j^T F K_i (with full intrinsics)
 * must have singular values (sigma, sigma, 0). The constructor folds the
 * principal points into G = K_j^T F K_i (focals deferred) and performs a single
 * SVD to precompute the coefficient vectors d_01, d_12; the per-iteration
 * residual is then a cheap closed form in (f_i, f_j) with analytic Jacobians.
 *
 * Variables are the two focal lengths, each a scalar double. Principal points are
 * constant for the edge and live in the constructor, not in the optimized state.
 */
class SelfCalibrationFactor
    : public NoiseModelFactorT<Vector2, double, double> {
 public:
  using Base = NoiseModelFactorT<Vector2, double, double>;
  using This = SelfCalibrationFactor;

 private:
  // Residual coefficients, precomputed from G = K_j^T F K_i in the constructor. They are a
  // function of F and the principal points only (not the focals), so they stay fixed for the
  // edge — the focal dependence is entirely in evaluateError.
  Eigen::Vector4d d_01_;
  Eigen::Vector4d d_12_;

 public:
  /**
   * @param fi_key Key of the focal length of camera i (F's right index).
   * @param fj_key Key of the focal length of camera j (F's left index).
   * @param F      3x3 fundamental matrix with convention x_j^T F x_i = 0.
   * @param pp_i   Principal point of camera i.
   * @param pp_j   Principal point of camera j.
   * @param model  Optional 2-dimensional noise model (e.g. robust Cauchy);
   *               defaults to unit noise when null.
   */
  SelfCalibrationFactor(Key fi_key, Key fj_key, const Matrix3& F, const Vector2& pp_i,
                        const Vector2& pp_j, const SharedNoiseModel& model = nullptr)
      : Base(model ? model : noiseModel::Unit::Create(2), fi_key, fj_key) {
    // Fold the principal points into G = K_j^T F K_i, leaving the focals out. K_j (the
    // transposed side) is F's left index, matching the convention x_j^T F x_i = 0.
    Matrix3 Ki = Matrix3::Identity();
    Ki(0, 2) = pp_i(0);
    Ki(1, 2) = pp_i(1);

    Matrix3 Kj = Matrix3::Identity();
    Kj(0, 2) = pp_j(0);
    Kj(1, 2) = pp_j(1);

    const Matrix3 G = Kj.transpose() * F * Ki;
    const std::array<Eigen::Vector4d, 3> ds = fetzer_ds(G);
    d_01_ = ds[0];
    d_12_ = ds[2];
  }

  /// 2-vector essential-matrix deviation; Jacobians are 2x1 each.
  Vector2 evaluateError(const double& fi, const double& fj,
                        OptionalMatrixType H1 = nullptr,
                        OptionalMatrixType H2 = nullptr) const override {
    // Floor the focal magnitude so a (near-)zero focal can't divide to inf/NaN; for any
    // realistic focal this is identical to the raw input.
    constexpr double kMinFocal = 1e-6;
    const double fi0 =
        std::abs(fi) < kMinFocal ? std::copysign(kMinFocal, fi) : fi;
    const double fj0 =
        std::abs(fj) < kMinFocal ? std::copysign(kMinFocal, fj) : fj;
    const double fi2 = fi0 * fi0;
    const double fj2 = fj0 * fj0;

    const Eigen::Vector4d& a = d_01_;
    const Eigen::Vector4d& b = d_12_;

    double di = a(0) * fj2 + a(1);
    double dj = b(0) * fi2 + b(2);
    // Floor the denominators so a rank-deficient F keeps K0/K1 finite.
    constexpr double kMinDenom = 1e-12;
    if (std::abs(di) < kMinDenom) di = std::copysign(kMinDenom, di);
    if (std::abs(dj) < kMinDenom) dj = std::copysign(kMinDenom, dj);

    const double ni = a(2) * fj2 + a(3);
    const double nj = b(1) * fi2 + b(3);

    const double K0 = -ni / di;  // f_i^2 implied by the f_j side
    const double K1 = -nj / dj;  // f_j^2 implied by the f_i side

    const double r0 = (fi2 - K0) / fi2;
    const double r1 = (fj2 - K1) / fj2;

    if (H1) {
      H1->resize(2, 1);
      (*H1)(0, 0) = 2.0 * (1.0 - r0) / fi0;
      (*H1)(1, 0) = (2.0 * fi0 / fj2) * (b(1) * dj - b(0) * nj) / (dj * dj);
    }
    if (H2) {
      H2->resize(2, 1);
      (*H2)(0, 0) = (2.0 * fj0 / fi2) * (a(2) * di - a(0) * ni) / (di * di);
      (*H2)(1, 0) = 2.0 * (1.0 - r1) / fj0;
    }

    return Vector{{r0, r1}};
  }
};

}  // namespace gtsam
